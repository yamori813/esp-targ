/*	$NetBSD: esp_pcmcia.c,v 1.39 2016/07/07 06:55:42 msaitoh Exp $	*/

/*-
 * Copyright (c) 2000, 2004 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Charles M. Hannum.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*	$NecBSD: ncr53c500_pisa.c,v 1.28 1998/11/26 01:59:11 honda Exp $	*/
/*	$NetBSD$	*/

/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * [Ported for FreeBSD]
 *  Copyright (c) 2000
 *      Noriaki Mitsunaga, Mitsuru Iwasaki and Takanori Watanabe.
 *      All rights reserved.
 * [NetBSD for NEC PC-98 series]
 *  Copyright (c) 1995, 1996, 1997, 1998
 *	NetBSD/pc98 porting staff. All rights reserved.
 *  Copyright (c) 1995, 1996, 1997, 1998
 *	Naofumi HONDA. All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: releng/12.2/sys/dev/esp/ncr53c500_pccard.c 328523 2018-01-29 00:14:46Z imp $");

#include <sys/param.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/scsi/scsi_all.h>

#include <sys/bus.h>

#include <dev/pccard/pccardvar.h>

#include <dev/esp/ncr53c9xreg.h>
#include <dev/esp/ncr53c9xvar.h>

#include "pccarddevs.h"

#define FIFO_F_SZ		128

#define	NCVIOSZ			0x10

struct esp_pcmcia_softc {
	struct ncr53c9x_softc	sc_ncr53c9x;	/* glue to MI code */
	device_t		sc_dev;

#define	ESP_PCCARD_RES_INTR	0
#define	ESP_PCCARD_RES_IO	1
	struct resource *sc_res[2];

	void			*sc_ih;		/* interrupt handler */

	int			sc_active;	/* Pseudo-DMA state vars */
	int			sc_tc;
	int			sc_datain;
	size_t			sc_dmasize;
	size_t			sc_dmatrans;
	void			**sc_dmaaddr;
	int			sc_offset;
	size_t			*sc_pdmalen;

};


uint8_t	esp_pcmcia_read_reg(struct ncr53c9x_softc *, int);
void	esp_pcmcia_write_reg(struct ncr53c9x_softc *, int, uint8_t);
int	esp_pcmcia_dma_isintr(struct ncr53c9x_softc *);
void	esp_pcmcia_dma_reset(struct ncr53c9x_softc *);
int	esp_pcmcia_dma_intr(struct ncr53c9x_softc *);
int	esp_pcmcia_dma_setup(struct ncr53c9x_softc *, void **,
	    size_t *, int, size_t *);
void	esp_pcmcia_dma_go(struct ncr53c9x_softc *);
void	esp_pcmcia_dma_stop(struct ncr53c9x_softc *);
int	esp_pcmcia_dma_isactive(struct ncr53c9x_softc *);

const struct ncr53c9x_glue esp_pcmcia_glue = {
	esp_pcmcia_read_reg,
	esp_pcmcia_write_reg,
	esp_pcmcia_dma_isintr,
	esp_pcmcia_dma_reset,
	esp_pcmcia_dma_intr,
	esp_pcmcia_dma_setup,
	esp_pcmcia_dma_go,
	esp_pcmcia_dma_stop,
	esp_pcmcia_dma_isactive,
};


static const struct esp_product {
	struct pccard_product	prod;
	int flags;
} esp_products[] = {
	{ PCMCIA_CARD(EPSON, SC200), 0},
	{ PCMCIA_CARD(PANASONIC, KXLC002), 0xb4d00000 },
	{ PCMCIA_CARD(PANASONIC, KXLC003), 0xb4d00000 },	/* untested */
	{ PCMCIA_CARD(PANASONIC, KXLC004), 0xb4d00100 },
	{ PCMCIA_CARD(MACNICA, MPS100), 0xb6250000 },
	{ PCMCIA_CARD(MACNICA, MPS110), 0 },
	{ PCMCIA_CARD(NEC, PC9801N_J03R), 0 },
	{ PCMCIA_CARD(NEWMEDIA, BASICS_SCSI), 0 },
	{ PCMCIA_CARD(QLOGIC, PC05), 0x84d00000 },
#define FLAGS_REX5572 0x84d00000
	{ PCMCIA_CARD(RATOC, REX5572), FLAGS_REX5572 },
	{ PCMCIA_CARD(RATOC, REX9530), 0x84d00000 },
	{ { NULL }, 0 }
};

/*
 * Additional code for FreeBSD new-bus PCCard frontend
 */

static void
esp_pccard_xfermap(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
}

static int
esp_pccard_probe(device_t dev)
{
	const struct esp_product *pp;
	const char *vendorstr;
	const char *prodstr;

	if ((pp = (const struct esp_product *) pccard_product_lookup(dev, 
	    (const struct pccard_product *) esp_products,
	    sizeof(esp_products[0]), NULL)) != NULL) {
		if (pp->prod.pp_name != NULL)
			device_set_desc(dev, pp->prod.pp_name);
		device_set_flags(dev, pp->flags);
		return(0);
	}
	if (pccard_get_vendor_str(dev, &vendorstr))
		return(EIO);
	if (pccard_get_product_str(dev, &prodstr))
		return(EIO);
	if (strcmp(vendorstr, "RATOC System Inc.") == 0 &&
		strncmp(prodstr, "SOUND/SCSI2 CARD", 16) == 0) {
		device_set_desc(dev, "RATOC REX-5572");
		device_set_flags(dev, FLAGS_REX5572);
		return (BUS_PROBE_DEFAULT);
	}
	return(EIO);
}

static int
esp_pccard_attach(device_t dev)
{
	struct esp_pcmcia_softc *esc;
	struct ncr53c9x_softc *sc;
	int error;
	rman_res_t ioaddr, iosize;
	bus_addr_t offset;
	int rid;

	offset = 0;

	esc = device_get_softc(dev);
	sc = &esc->sc_ncr53c9x;

	NCR_LOCK_INIT(sc);

	esc->sc_dev = dev;
	sc->sc_glue = &esp_pcmcia_glue;

	error = bus_get_resource(dev, SYS_RES_IOPORT, 0, &ioaddr, &iosize);
	if (error || (iosize < (offset + NCVIOSZ))) {
		return(ENOMEM);
	}

	rid = 0;
	esc->sc_res[ESP_PCCARD_RES_IO] = bus_alloc_resource(dev, SYS_RES_IOPORT,
	    &rid, ioaddr+offset, ioaddr+iosize-offset,
	    iosize-offset, RF_ACTIVE);

	sc->sc_rev = NCR_VARIANT_NCR53C500;
	sc->sc_id = 0;
	sc->sc_freq = 40;
	/* try -PARENB -SLOW */
//	sc->sc_cfg1 = sc->sc_id | NCRCFG1_PARENB | NCRCFG1_SLOW;
	sc->sc_cfg1 = sc->sc_id;
	/* try +FE */
//	sc->sc_cfg2 = NCRCFG2_FE | NCRCFG2_SCSI2;
	sc->sc_cfg2 = NCRCFG2_FE;
	/* try -IDM -FSCSI -FCLK */
//	sc->sc_cfg3 = NCRESPCFG3_CDB | NCRESPCFG3_FCLK | NCRESPCFG3_IDM |
//	    NCRESPCFG3_FSCSI;
	sc->sc_cfg3 = NCRESPCFG3_CDB | NCRESPCFG3_FSCSI;
	sc->sc_cfg4 = NCRCFG4_ACTNEG;
	/* try +INTP */
	sc->sc_cfg5 = NCRCFG5_CRS1 | NCRCFG5_AADDR | NCRCFG5_PTRINC;
	sc->sc_minsync = 1000 / sc->sc_freq;
	sc->sc_maxxfer = 64 * 1024;
	sc->sc_extended_geom = 1;

	rid = 0;
	esc->sc_res[ESP_PCCARD_RES_INTR] = bus_alloc_resource_any(dev,
	     SYS_RES_IRQ, &rid, RF_ACTIVE);

	if (esc->sc_res[ESP_PCCARD_RES_INTR] == NULL) {
		device_printf(dev, "couldn't map interrupt\n");
		error = ENXIO;
		goto fail;
	}

	error = bus_setup_intr(dev, esc->sc_res[ESP_PCCARD_RES_INTR],
	    INTR_MPSAFE | INTR_TYPE_CAM, NULL, ncr53c9x_intr, sc,
	    &esc->sc_ih);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail_intr;
	}

	/* Do the common parts of attachment. */
	sc->sc_dev = esc->sc_dev;
	error = ncr53c9x_attach(sc);
	if (error != 0) {
		device_printf(esc->sc_dev, "ncr53c9x_attach failed\n");
		goto fail_intr;
	}

	return (0);

 fail_intr:
	 bus_teardown_intr(esc->sc_dev, esc->sc_res[ESP_PCCARD_RES_INTR],
	    esc->sc_ih);

 fail:
	NCR_LOCK_DESTROY(sc);

	return (error);
}

static	int
esp_pccard_detach(device_t dev)
{
	struct esp_pcmcia_softc *esc = device_get_softc(dev);
	int error;

	error = ncr53c9x_detach(&esc->sc_ncr53c9x);
	if (error)
		return error;

	return (0);
}

static device_method_t esp_pccard_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		esp_pccard_probe),
	DEVMETHOD(device_attach,	esp_pccard_attach),
	DEVMETHOD(device_detach,	esp_pccard_detach),

	{ 0, 0 }
};

static driver_t esp_pccard_driver = {
	"esp",
	esp_pccard_methods,
	sizeof(struct ncr53c9x_softc),
};

DRIVER_MODULE(esp, pccard, esp_pccard_driver, esp_devclass, 0, 0);
PCCARD_PNP_INFO(esp_products);


/*
 * Glue functions.
 */
uint8_t
esp_pcmcia_read_reg(struct ncr53c9x_softc *sc, int reg)
{
	struct esp_pcmcia_softc *esc = (struct esp_pcmcia_softc *)sc;

	return bus_read_1((esc)->sc_res[ESP_PCCARD_RES_IO], reg);
}

void
esp_pcmcia_write_reg(struct ncr53c9x_softc *sc, int reg, uint8_t val)
{
	struct esp_pcmcia_softc *esc = (struct esp_pcmcia_softc *)sc;
	uint8_t v = val;

	if (reg == NCR_CMD && v == (NCRCMD_TRANS|NCRCMD_DMA))
		v = NCRCMD_TRANS;
	bus_write_1((esc)->sc_res[ESP_PCCARD_RES_IO], reg, (val));
}

int
esp_pcmcia_dma_isintr(struct ncr53c9x_softc *sc)
{
int pstat;

//	NCR_WRITE_REG(sc, NCR_CFG5, sc->sc_cfg5 | NCRCFG5_SINT);
	NCR_WRITE_REG(sc, NCR_CFG5, sc->sc_cfg5);
	pstat = NCR_READ_REG(sc, NCR_PSTAT);
//	device_printf(sc->sc_dev, "TARG ISINTR %x\n", pstat);
	NCR_WRITE_REG(sc, NCR_CFG4, sc->sc_cfg4);

	return NCR_READ_REG(sc, NCR_STAT) & NCRSTAT_INT;
}

void
esp_pcmcia_dma_reset(struct ncr53c9x_softc *sc)
{
	struct esp_pcmcia_softc *esc = (struct esp_pcmcia_softc *)sc;

	esc->sc_active = 0;
	esc->sc_tc = 0;
}

int
esp_pcmcia_dma_intr(struct ncr53c9x_softc *sc)
{

	/* use buzy loop */

	return 0;
}

int
esp_pcmcia_dma_setup(struct ncr53c9x_softc *sc, void **addr, size_t *len,
    int datain, size_t *dmasize)
{
	struct esp_pcmcia_softc *esc = (struct esp_pcmcia_softc *)sc;
	int pstat;
	int reqlen;
	u_int8_t *ptr;

	esc->sc_datain = datain;
	esc->sc_dmaaddr = addr;
	esc->sc_pdmalen = len;    /* This is start size */
	esc->sc_dmasize = *dmasize;
	esc->sc_tc = 0;
	esc->sc_offset = 0;

//	NCR_WRITE_REG(sc, NCR_CFG5, sc->sc_cfg5 | NCRCFG5_SINT);
	NCR_WRITE_REG(sc, NCR_CFG5, sc->sc_cfg5);

	pstat = NCR_READ_REG(sc, NCR_PSTAT);
//	device_printf(sc->sc_dev, "TARG SETUP %x\n", pstat);
	if (esc->sc_datain == 0) {
		if (esc->sc_dmasize > FIFO_F_SZ) {
			bus_write_multi_4(
			    (esc)->sc_res[ESP_PCCARD_RES_IO],
			    NCR_PIOFIFO, (u_int32_t *) *addr,
			    FIFO_F_SZ / 4);
			esc->sc_offset = FIFO_F_SZ;
			esc->sc_dmasize -= FIFO_F_SZ;
		} else {
			reqlen = esc->sc_dmasize;
			ptr = *esc->sc_dmaaddr;
			while(reqlen > 0) {
				bus_write_1((esc)->sc_res[ESP_PCCARD_RES_IO],
				    NCR_PIOFIFO, *ptr);

				++ptr;
				--reqlen;
			}
			esc->sc_dmasize = 0;
		}
	}

	NCR_WRITE_REG(sc, NCR_CFG4, sc->sc_cfg4);

	return 0;
}

void
esp_pcmcia_dma_go(struct ncr53c9x_softc *sc)
{
	struct esp_pcmcia_softc *esc = (struct esp_pcmcia_softc *)sc;
	int size;
	int reqlen;
	u_int8_t *ptr;
	int pstat, lastst;
	int count;
	
//	NCR_WRITE_REG(sc, NCR_CFG5, sc->sc_cfg5 | NCRCFG5_SINT);
	NCR_WRITE_REG(sc, NCR_CFG5, sc->sc_cfg5);

	reqlen = esc->sc_dmasize;
	ptr = *esc->sc_dmaaddr;
	ptr += esc->sc_offset;

	if (esc->sc_datain) {
		count = 0;
		while(reqlen > 1) {
			size = FIFO_F_SZ;

			pstat = NCR_READ_REG(sc, NCR_PSTAT);
			if (pstat & NCRPSTAT_PERR || count > 5000) {
				device_printf(sc->sc_dev, "TARG PIO ERR R %d %d %02x\n",
				    *esc->sc_pdmalen, reqlen, pstat);
				reqlen = 0;
				break;
			}
			++count;

			if ((pstat & NCRPSTAT_FFULL) != 0) {
				bus_read_multi_4((esc)->sc_res[ESP_PCCARD_RES_IO], NCR_PIOFIFO,
				    (u_int32_t *) ptr, size / 4);

				reqlen -= size;
				ptr += size;
				count = 0;
			} 

			DELAY(1);
		}
	} else {
		count = 0;
		/* fast time wait */
		if (reqlen == *esc->sc_pdmalen - FIFO_F_SZ)
			DELAY(10);

		while(reqlen >= FIFO_F_SZ) {
			pstat = NCR_READ_REG(sc, NCR_PSTAT);
			/* 0x00 is ??? */
			if (pstat & NCRPSTAT_PERR || pstat == 0x00 || count > 5000) {
				device_printf(sc->sc_dev, "TARG PIO ERR W %d %d %02x\n",
				    *esc->sc_pdmalen, reqlen, pstat);
				reqlen = 0;
				break;
			}
			++count;

			if (pstat & NCRPSTAT_FEMPT) {
				size = FIFO_F_SZ;
				bus_write_multi_4(
				    (esc)->sc_res[ESP_PCCARD_RES_IO],
				    NCR_PIOFIFO, (u_int32_t *) ptr,
				    size / 4);
				ptr += size;
				reqlen -= size;
				count = 0;
			} else
				DELAY(1);

			lastst = pstat;
		}
	}


//	esc->sc_active = 1;

}

void
esp_pcmcia_dma_stop(struct ncr53c9x_softc *sc)
{
	struct esp_pcmcia_softc *esc = (struct esp_pcmcia_softc *)sc;

	NCR_WRITE_REG(sc, NCR_CFG4, sc->sc_cfg4);
	esc->sc_active = 0;
}

int
esp_pcmcia_dma_isactive(struct ncr53c9x_softc *sc)
{
	struct esp_pcmcia_softc *esc = (struct esp_pcmcia_softc *)sc;

	return esc->sc_active;
}

