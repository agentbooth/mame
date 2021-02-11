// license:GPL-2.0+
// copyright-holders:Dirk Best, R. Belmont
/***************************************************************************

    AT&T UNIX PC series (7300 and 3B1)

    Skeleton driver by Dirk Best and R. Belmont

    DIVS instruction at 0x801112 (the second time) causes a divide-by-zero
    exception the system isn't ready for due to word at 0x5EA6 being zero.

    Code might not get there if the attempted FDC boot succeeds; FDC hookup
    probably needs help.  2797 isn't asserting DRQ?

***************************************************************************/


#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "bus/centronics/ctronics.h"
#include "bus/rs232/rs232.h"
#include "imagedev/floppy.h"
#include "imagedev/harddriv.h"
#include "machine/6850acia.h"
#include "machine/74259.h"
#include "machine/bankdev.h"
#include "machine/input_merger.h"
#include "machine/output_latch.h"
#include "machine/ram.h"
//#include "machine/tc8250.h"
#include "machine/wd1010.h"
#include "machine/wd_fdc.h"
#include "machine/z80sio.h"
#include "emupal.h"
#include "screen.h"

#include "unixpc.lh"


/***************************************************************************
    DRIVER STATE
***************************************************************************/

class unixpc_state : public driver_device
{
public:
	unixpc_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_gcr(*this, "gcr"),
		m_tcr(*this, "tcr"),
		m_int02(*this, "int02"),
		m_ram(*this, RAM_TAG),
		m_fdc(*this, "wd2797"),
		m_floppy(*this, "wd2797:0:525dd"),
		m_hdc(*this, "hdc"),
		m_hdr0(*this, "hdc:0"),
		m_ramrombank(*this, "ramrombank"),
		m_mapram(*this, "mapram"),
		m_videoram(*this, "videoram")
	{ }

	void unixpc(machine_config &config);

private:
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	virtual void machine_start() override;
	virtual void machine_reset() override;

	uint16_t line_printer_r();
	void disk_control_w(uint8_t data);
	void gcr_w(offs_t offset, uint16_t data);
	DECLARE_WRITE_LINE_MEMBER(romlmap_w);
	DECLARE_WRITE_LINE_MEMBER(error_enable_w);
	DECLARE_WRITE_LINE_MEMBER(parity_enable_w);
	DECLARE_WRITE_LINE_MEMBER(bpplus_w);
	DECLARE_WRITE_LINE_MEMBER(dma_reading_w);
	uint16_t ram_mmu_r(offs_t offset);
	void ram_mmu_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	uint16_t gsr_r();
	void tcr_w(offs_t offset, uint16_t data);
	uint16_t tsr_r();
	uint16_t rtc_r();
	void rtc_w(uint16_t data);
	void csr_w(uint16_t data);
	uint16_t diskdma_size_r();
	void diskdma_size_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void diskdma_ptr_w(offs_t offset, uint16_t data);

	DECLARE_WRITE_LINE_MEMBER(wd2797_intrq_w);
	DECLARE_WRITE_LINE_MEMBER(wd2797_drq_w);

	DECLARE_WRITE_LINE_MEMBER(wd1010_intrq_w);

	void ramrombank_map(address_map &map);
	void unixpc_mem(address_map &map);

private:
	required_device<cpu_device> m_maincpu;
	required_device<ls259_device> m_gcr;
	required_device<ls259_device> m_tcr;
	required_device<input_merger_device> m_int02;
	required_device<ram_device> m_ram;
	required_device<wd2797_device> m_fdc;
	required_device<floppy_image_device> m_floppy;
	required_device<wd1010_device> m_hdc;
	required_device<harddisk_image_device> m_hdr0;
	required_device<address_map_bank_device> m_ramrombank;

	required_shared_ptr<uint16_t> m_mapram;
	required_shared_ptr<uint16_t> m_videoram;

	uint16_t *m_ramptr;
	uint32_t m_ramsize;
	uint16_t m_diskdmacount;
	uint32_t m_diskdmaptr;	 // aka DMA address

	// DMA direction
	bool		m_idmarw;
	// DMA enable
	bool		m_dmaen;

	// Registers
	uint16_t	m_gsr;
	uint16_t	m_bsr0;
	uint16_t	m_bsr1;

	bool m_fdc_intrq;
	bool m_hdc_intrq;
};


/***************************************************************************
    MEMORY
***************************************************************************/

void unixpc_state::gcr_w(offs_t offset, uint16_t data)
{
	// calls the individual bit handlers (romlmap_w, error_enable_w, ...)
	m_gcr->write_bit(offset >> 11, BIT(data, 15));
}

WRITE_LINE_MEMBER(unixpc_state::romlmap_w)
{
	logerror("romlmap_w: %d\n", state);
	m_ramrombank->set_bank(state ? 1 : 0);
}

//#define COMBINE_DATA(varptr)            (*(varptr) = (*(varptr) & ~mem_mask) | (data & mem_mask))

static constexpr unsigned MMU_MAX_PAGES = 1024;
static constexpr uint16_t MMU_WRITE_ENABLE = 0x8000;
static constexpr uint16_t MMU_STATUS_MASK  = 0x6000;
static constexpr uint16_t MMU_STATUS_NOT_PRESENT = 0x0000;
static constexpr uint16_t MMU_STATUS_PRESENT_NOT_ACCESSED = 0x2000;
static constexpr uint16_t MMU_STATUS_ACCESSED_NOT_WRITTEN = 0x4000;
static constexpr uint16_t MMU_STATUS_ACCESSED_WRITTEN = 0x6000;

uint16_t unixpc_state::ram_mmu_r(offs_t offset)
{
	uint16_t page = (offset >> 11) & 0x3ff;
	uint16_t mapentry = m_mapram[page];

#if 0
	uint8_t fc = m_maincpu->get_fc();
	// only supervisor (kernel) can access low 512KB virtual address space
	if ((offset < ((512*1024)>>1)) && (fc != M68K_FC_SUPERVISOR_DATA) && (fc != M68K_FC_SUPERVISOR_PROGRAM))
	{
		// need to throw MEM_KERNEL error
		fatalerror("mmu: user mode access to lower 512K, need to generate a fault\n");
	}
#endif

	if ((mapentry & MMU_STATUS_MASK) == MMU_STATUS_NOT_PRESENT)
	{
		// need to throw page fault -- "else" statement should probably not be conditional
		logerror("PAGEFAULT - virtual addr [0x%06x] read, page 0x%x\n", offset, page);
		return 0xFFFF;
	}
	else
	{
		uint32_t addr = (offset & 0x7ff) | ((mapentry & 0x3ff) << 11);
		//logerror("virtual addr [0x%06x] read, data = %04x\n", offset << 1, m_ramptr[addr]);
		//logerror("mmu_r: mapentry %04x translated to phys addr 0x%06x\n", mapentry, addr);

		// indicate page has been read
		if ((mapentry & MMU_STATUS_MASK) == MMU_STATUS_PRESENT_NOT_ACCESSED)
		{
			m_mapram[page] &= ~MMU_STATUS_MASK;
			m_mapram[page] |= MMU_STATUS_ACCESSED_NOT_WRITTEN;
		}
		
		return m_ramptr[addr];
	}
}

void unixpc_state::ram_mmu_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t page = (offset >> 11) & 0x3ff;
	uint16_t mapentry = m_mapram[page];

	//logerror("virtual addr [0x%06x] write = %04x (mem_mask %04x)\n", offset << 1, data, mem_mask);
	
#if 0
	uint8_t fc = m_maincpu->get_fc();
	// only supervisor (kernel) can access low 512KB virtual address space
	if ((offset < ((512*1024)>>1)) && (fc != M68K_FC_SUPERVISOR_DATA) && (fc != M68K_FC_SUPERVISOR_PROGRAM))
	{
		// need to throw MEM_KERNEL error
		fatalerror("mmu: user mode access to lower 512K, need to generate a fault\n");
	}
#endif

	if ((mapentry & MMU_STATUS_MASK) == MMU_STATUS_NOT_PRESENT)
	{
		// need to throw page fault -- "else" statement should probably not be conditional
		logerror("PAGEFAULT - virtual addr [0x%06x] read, page 0x%x\n", offset, page);
	}
	else
	{
		uint32_t addr = (offset & 0x7ff) | ((mapentry & 0x3ff) << 11);
		//logerror("mmu_w: mapentry %04x translated to phys addr 0x%06x\n", mapentry, addr);
#if 0
		// check Write Enable page bit, this can actually just be ignored in FreeBee
		if (!(mapentry & MMU_WRITE_ENABLE) && (fc != M68K_FC_SUPERVISOR_PROGRAM) && (fc != M68K_FC_SUPERVISOR_DATA))
		{
			fatalerror("mmu: write protection violation, need to throw a fault\n");
		}
#endif
		// indicate page has been written
		m_mapram[page] |= MMU_STATUS_ACCESSED_WRITTEN;

		COMBINE_DATA(&m_ramptr[addr]);
		//m_ramptr[addr] = (m_ramptr[addr] & ~mem_mask) | (data & mem_mask);
		//m_ramptr[addr] = (m_ramptr[addr] & ~0xFFFF) | (data & 0xFFFF);
		//m_ramptr[addr] = data;
	}
}

void unixpc_state::machine_start()
{
	m_ramptr = (uint16_t *)m_ram->pointer();
	m_ramsize = m_ram->size();
}

void unixpc_state::machine_reset()
{
	//disk_control_w(0);
	
	// double density (MFM) enable
	m_fdc->dden_w(1);
}

WRITE_LINE_MEMBER(unixpc_state::error_enable_w)
{
	logerror("error_enable_w: %d\n", state);
}

WRITE_LINE_MEMBER(unixpc_state::parity_enable_w)
{
	logerror("parity_enable_w: %d\n", state);
}

WRITE_LINE_MEMBER(unixpc_state::bpplus_w)
{
	logerror("bpplus_w: %d\n", state);
}

WRITE_LINE_MEMBER(unixpc_state::dma_reading_w)
{
	logerror("dma_reading: %d\n", state);
}


/***************************************************************************
    MISC
***************************************************************************/

uint16_t unixpc_state::gsr_r()
{
	// needs to be updated appropriately when pagefault occurs
	return m_gsr;
}

void unixpc_state::tcr_w(offs_t offset, uint16_t data)
{
	m_tcr->write_bit(offset >> 11, BIT(data, 14));
}

uint16_t unixpc_state::tsr_r()
{
	return 0;
}

uint16_t unixpc_state::rtc_r()
{
	return 0;
}

void unixpc_state::rtc_w(uint16_t data)
{
	logerror("rtc_w: %04x\n", data);
}

void unixpc_state::csr_w(uint16_t data)
{
	logerror("csr_w: %04x\n", data);
	m_gsr = 0xFFFF;
	m_bsr0 = 0xFFFF;
	m_bsr1 = 0xFFFF;
}

uint16_t unixpc_state::line_printer_r()
{
	uint16_t data = 0;

	data |= 1; // no dial tone detected
	data |= 1 << 1; // no parity error
	data |= m_hdc_intrq ? 1<<2 : 0<<2;
	data |= m_fdc_intrq ? 1<<3 : 0<<3;
	data |= 1 << 4; // no line printer error

	//logerror("line_printer_r: %04x\n", data);

	return data;
}

/***************************************************************************
    DMA
***************************************************************************/

uint16_t unixpc_state::diskdma_size_r()
{
	return (m_diskdmacount & 0x3FFF) | 0xC000;
}

void unixpc_state::diskdma_size_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// 2's complement of number of words to be DMA'd, counts until 0x3FFF
	m_diskdmacount = data & 0x3FFF;
	// advance DMA count to simulate advancing past false DMA transfer
	m_diskdmacount++;
	
	m_idmarw = ((data & 0x4000) == 0x4000);  // 1 = DMA read, 0 = DMA write
	m_dmaen = ((data & 0x8000) == 0x8000);   // Each time this goes from 0 to 1, DMA disk request is generated
	logerror("disk DMA count %x, dmaen: %i, DMA read: %i\n", m_diskdmacount, m_dmaen, m_idmarw );	
}

void unixpc_state::diskdma_ptr_w(offs_t offset, uint16_t data)
{
	if (offset >= 0x2000)
	{
		// set top 4 bytes
		//m_diskdmaptr &= 0xff;
		//m_diskdmaptr |= (offset << 8);
		m_diskdmaptr = (m_diskdmaptr & 0x1FE) | ((offset<<1 & 0x3FFE) << 8);
	}
	else
	{
		//m_diskdmaptr &= 0xffff00;
		//m_diskdmaptr |= (offset & 0xff);
		m_diskdmaptr = (m_diskdmaptr & 0x3FFE00) | (offset<<1 & 0x1FE);
	}
	logerror("diskdma_ptr_w: wrote at %x, DMA address now 0x%x\n", offset<<1, m_diskdmaptr);
}

/***************************************************************************
    FLOPPY
***************************************************************************/

void unixpc_state::disk_control_w(uint8_t data)
{
	//logerror("disk_control_w: %02x\n", data);
	logerror("fdc reset: %i, hdc reset: %i, floppy selected: %i, floppy motor: %i, HD selected: %i\n", !BIT(data, 7), !BIT(data, 4), BIT(data, 6), BIT(data, 5), BIT(data,3));

	// TODO: bits 0-2 = HD head select
	//uint8_t sdh = wd2010_read_reg(WD2010_REG_SDH);
	//sdh = (sdh & ~0x07) | (data & 0x07);
	//wd2010_write_reg(WD2010_REG_SDH, sdh);
	
	// bit 3 = HDD0 select (hd_selected)
	m_hdc->drdy_w(BIT(data, 3) && m_hdr0->exists());

	// bit 4 = hdc reset (when == 0)
	if (!BIT(data, 4))
		m_hdc->reset();

	// bit 5 = floppy image, motor enable (when == 1)
	m_floppy->mon_w(!BIT(data, 5));

	// bit 6 = floppy select (fd_selected)
	if (BIT(data, 6))
		m_fdc->set_floppy(m_floppy);
	else
		m_fdc->set_floppy(nullptr);

	// bit 7 = fdc reset (when == 0), send to Master Reset- (MR-)
	m_fdc->mr_w(BIT(data, 7));
}

WRITE_LINE_MEMBER(unixpc_state::wd2797_intrq_w)
{
	logerror("wd2797_intrq_w: %d, drq: %i\n", state, m_fdc->drq_r());
	m_fdc_intrq = state;
	m_int02->in_w<1>(state);
}

WRITE_LINE_MEMBER(unixpc_state::wd2797_drq_w)
{
	logerror("wd2797_drq_w: %d\n", state);
	uint16_t data_read;
	if (state)
	{
		data_read = m_fdc->data_r();
		data_read <<= 8;
		data_read += m_fdc->data_r();
		logerror("floppy read attempt, DMA count: %x, DMA address: 0x%x, data read: %04x\n", m_diskdmacount, m_diskdmaptr, data_read);
		// TODO: MMU map, m_diskdmaptr needs to be mapped to physical address
		m_ramptr[m_diskdmaptr] = data_read;
		// Increment DMA address
		m_diskdmaptr += 2;
		// Increment number of words transferred
		m_diskdmacount++;
	}
}

/***************************************************************************
    HARD DISK
***************************************************************************/

WRITE_LINE_MEMBER(unixpc_state::wd1010_intrq_w)
{
	m_hdc_intrq = state;
	m_int02->in_w<0>(state);
}

/***************************************************************************
    VIDEO
***************************************************************************/

uint32_t unixpc_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for (int y = 0; y < 348; y++)
		for (int x = 0; x < 720/16; x++)
			for (int b = 0; b < 16; b++)
				bitmap.pix(y, x * 16 + b) = BIT(m_videoram[y * (720/16) + x], b);

	return 0;
}


/***************************************************************************
    ADDRESS MAPS
***************************************************************************/

void unixpc_state::unixpc_mem(address_map &map)
{
	map(0x000000, 0x3fffff).m(m_ramrombank, FUNC(address_map_bank_device::amap16));
	map(0x400000, 0x4007ff).ram().share("mapram");
	map(0x410000, 0x410001).r(FUNC(unixpc_state::gsr_r));
	map(0x420000, 0x427fff).ram().share("videoram");
	map(0x450000, 0x450001).r(FUNC(unixpc_state::tsr_r));
	map(0x460000, 0x460001).rw(FUNC(unixpc_state::diskdma_size_r), FUNC(unixpc_state::diskdma_size_w));
	map(0x470000, 0x470001).r(FUNC(unixpc_state::line_printer_r));
	map(0x480000, 0x480001).w(FUNC(unixpc_state::rtc_w));
	map(0x490000, 0x490001).select(0x7000).w(FUNC(unixpc_state::tcr_w));
	map(0x4a0000, 0x4a0000).w("mreg", FUNC(output_latch_device::write));
	map(0x4c0000, 0x4c0001).w(FUNC(unixpc_state::csr_w));
	map(0x4d0000, 0x4d7fff).w(FUNC(unixpc_state::diskdma_ptr_w));
	map(0x4e0001, 0x4e0001).w(FUNC(unixpc_state::disk_control_w)).cswidth(16);
	map(0x4f0001, 0x4f0001).w("printlatch", FUNC(output_latch_device::write));
	map(0xe00000, 0xe0000f).rw(m_hdc, FUNC(wd1010_device::read), FUNC(wd1010_device::write)).umask16(0x00ff);
	map(0xe10000, 0xe10007).rw(m_fdc, FUNC(wd2797_device::read), FUNC(wd2797_device::write)).umask16(0x00ff);
	map(0xe30000, 0xe30001).r(FUNC(unixpc_state::rtc_r));
	map(0xe40000, 0xe40001).select(0x7000).w(FUNC(unixpc_state::gcr_w));
	map(0xe50000, 0xe50007).rw("mpsc", FUNC(upd7201_device::cd_ba_r), FUNC(upd7201_device::cd_ba_w)).umask16(0x00ff);
	map(0xe70000, 0xe70003).rw("kbc", FUNC(acia6850_device::read), FUNC(acia6850_device::write)).umask16(0xff00);
	map(0x800000, 0x803fff).mirror(0x7fc000).rom().region("bootrom", 0);
}

void unixpc_state::ramrombank_map(address_map &map)
{
	map(0x000000, 0x3fffff).rom().region("bootrom", 0);
	map(0x400000, 0x7fffff).rw(FUNC(unixpc_state::ram_mmu_r), FUNC(unixpc_state::ram_mmu_w));
}

/***************************************************************************
    INPUT PORTS
***************************************************************************/

static INPUT_PORTS_START( unixpc )
INPUT_PORTS_END


/***************************************************************************
    MACHINE DRIVERS
***************************************************************************/

static void unixpc_floppies(device_slot_interface &device)
{
	device.option_add("525dd", FLOPPY_525_DD);
}

void unixpc_state::unixpc(machine_config &config)
{
	// basic machine hardware
	M68010(config, m_maincpu, 40_MHz_XTAL / 4);
	m_maincpu->set_addrmap(AS_PROGRAM, &unixpc_state::unixpc_mem);

	LS259(config, m_gcr); // 7K
	m_gcr->q_out_cb<0>().set(FUNC(unixpc_state::error_enable_w));
	m_gcr->q_out_cb<1>().set(FUNC(unixpc_state::parity_enable_w));
	m_gcr->q_out_cb<2>().set(FUNC(unixpc_state::bpplus_w));
	m_gcr->q_out_cb<3>().set(FUNC(unixpc_state::romlmap_w));

	LS259(config, m_tcr); // 10K

	INPUT_MERGER_ANY_HIGH(config, m_int02); // 26H pins 3-6
	m_int02->output_handler().set_inputline(m_maincpu, M68K_IRQ_2);

	output_latch_device &mreg(OUTPUT_LATCH(config, "mreg"));
	mreg.bit_handler<0>().set_output("led_0").invert();
	mreg.bit_handler<1>().set_output("led_1").invert();
	mreg.bit_handler<2>().set_output("led_2").invert();
	mreg.bit_handler<3>().set_output("led_3").invert();
	// bit 4 (D12) = 0 = modem baud rate from UART clock inputs, 1 = baud from programmable timer
	mreg.bit_handler<5>().set("printer", FUNC(centronics_device::write_strobe)).invert();
	// bit 6 (D14) = 0 for disk DMA write, 1 for disk DMA read
	mreg.bit_handler<6>().set(FUNC(unixpc_state::dma_reading_w));
	// bit 7 (D15) = VBL ack (must go high-low-high to ack)

	// video hardware
	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	screen.set_screen_update(FUNC(unixpc_state::screen_update));
	screen.set_raw(40_MHz_XTAL / 2, 896, 0, 720, 367, 0, 348);
	screen.set_palette("palette");
	// vsync should actually last 17264 pixels

	config.set_default_layout(layout_unixpc);

	PALETTE(config, "palette", palette_device::MONOCHROME);

	// internal ram
	RAM(config, RAM_TAG).set_default_size("1M").set_extra_options("2M");

	// RAM/ROM bank
	ADDRESS_MAP_BANK(config, "ramrombank").set_map(&unixpc_state::ramrombank_map).set_options(ENDIANNESS_BIG, 16, 32, 0x400000);

	// floppy
	WD2797(config, m_fdc, 40_MHz_XTAL / 40); // 1PCK (CPU clock) divided by custom DMA chip
	m_fdc->intrq_wr_callback().set(FUNC(unixpc_state::wd2797_intrq_w));
	m_fdc->drq_wr_callback().set(FUNC(unixpc_state::wd2797_drq_w));
	FLOPPY_CONNECTOR(config, "wd2797:0", unixpc_floppies, "525dd", floppy_image_device::default_floppy_formats);

	WD1010(config, m_hdc, 40_MHz_XTAL / 8);
	m_hdc->out_intrq_callback().set(FUNC(unixpc_state::wd1010_intrq_w));
	HARDDISK(config, m_hdr0, 0);

	upd7201_device &mpsc(UPD7201(config, "mpsc", 19.6608_MHz_XTAL / 8));
	mpsc.out_txda_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	mpsc.out_dtra_callback().set("rs232", FUNC(rs232_port_device::write_dtr));
	mpsc.out_rtsa_callback().set("rs232", FUNC(rs232_port_device::write_rts));
	mpsc.out_int_callback().set_inputline(m_maincpu, M68K_IRQ_4);

	acia6850_device &kbc(ACIA6850(config, "kbc", 0));
	kbc.irq_handler().set_inputline(m_maincpu, M68K_IRQ_3);

	// TODO: RTC
	//TC8250(config, "rtc", 32.768_kHz_XTAL);

	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, nullptr));
	rs232.rxd_handler().set("mpsc", FUNC(upd7201_device::rxa_w));
	rs232.dsr_handler().set("mpsc", FUNC(upd7201_device::dcda_w));
	rs232.cts_handler().set("mpsc", FUNC(upd7201_device::ctsa_w));

	centronics_device &printer(CENTRONICS(config, "printer", centronics_devices, nullptr));
	output_latch_device &printlatch(OUTPUT_LATCH(config, "printlatch"));
	printer.set_output_latch(printlatch);
}


/***************************************************************************
    ROM DEFINITIONS
***************************************************************************/

// ROMs were provided by Michael Lee und imaged by Philip Pemberton
ROM_START( 3b1 )
	ROM_REGION16_BE(0x400000, "bootrom", 0)
	ROM_LOAD16_BYTE("72-00617.15c", 0x000000, 0x002000, CRC(4e93ff40) SHA1(1a97c8d32ec862f7f5fa1032f1688b76ea0672cc))
	ROM_LOAD16_BYTE("72-00616.14c", 0x000001, 0x002000, CRC(c61f7ae0) SHA1(ab3ac29935a2a587a083c4d175a5376badd39058))
ROM_END


/***************************************************************************
    GAME DRIVERS
***************************************************************************/

//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT        COMPANY  FULLNAME             FLAGS
COMP( 1985, 3b1,  0,      0,      unixpc,  unixpc, unixpc_state, empty_init, "AT&T",  "UNIX PC Model 3B1", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
