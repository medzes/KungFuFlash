/*
 * Copyright (c) 2021 Marko Edzes
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

/* 
 * The code below is inspired heavily by the document
 *  "Designing a Real KERNAL Cartridge" by Thomas ’skoe’ Giesel, version 1.1 Nov 4, 2015.
 * 
 * The document describes a trick forcing A14 low for a fraction of a cycle
 * after every write to $01 to find out the state of the "HIRAM" signal.
 * The code below follows the timing and implementation as described in the doc.
 * This driver also handles the easyflash de00/df00 and easyflash menu cartridge code.
 * That allows the soft-kernal handler to allow loading/saving from SDcard.
 * 
 * This code was developed on a DEVEBOX prototype of the KungFuFlash. 
 * The ARM CPU is identical to the KungFuFlash rev 2 PCB but the pinout is different.
 * This should not make much difference, but gcc will do some funny constant optimizations 
 * that can affect the delay of the code in very unexpected ways.
 * 
 * There is one noteworthy change for soft-kernal operation:
 *  - EXROM and GAME speed can be slower on the rev.2 PCB because they are on PC14,PC15. (On DEVEBOX they are on PD0,PD1)
 *    PC14,15 are special low-power mode pins that have a weak driver to +3.3V but strong drive to 0V.
 *    The EXROM/GAME rising edge is important to meet the timing during the KERNAL_CHECK mode below.
 * 
 * There are four very critical code paths through this code:
 *  - Address valid to case decode. Address is valid before all control is valid.
 *  - Control valid to case decode. BA/WRITE are valid before ROMH/ROML/IO1/IO2
 *  - A14 check: From 280ns after phi2 rise to data output
 *  - $de02 write: extends into next cycle. Needs to be heavily optimized.
 * 
 * ARM cycle accuracy
 * 
 * Getting the code cycle-accurate on ARM is very difficult. 
 * It appears the many busses the ARM CPU uses internally, with different access speeds cause
 * complex interaction. In one test, a single-cycle offset at the start
 *  of the interrupt was multiplied to 18 cycle difference at the end of the interrupt.
 *
 * The ARM seems to access peripherals on a clock/2 or clock/4 grid, rounding up accesses to the nearest edge.
 * The Cortext-M4 has a speculative branch pre-fetch, which happens many
 * times in the routine - on every conditional branch, so after every if() expression in C.
 * The delay of the branch instruction when not-taken is 1 cycle. 
 * So the most critical code should have the most not-taken branches.
 * 
 * Interrupt latency is hard to measure. The delay between handlers is easier to measure leaving the DWT-CYCCNT free-running.
 * During interrupt-to-interrupt delay a delta of -3 to +2 has been observed.
 * That would mean latency of a single interrupt can be +0 to +2 running some main code.
 * But if the interrupt occurs while main code does a a slow APB it could be 3 or flash read it might be 6.
 * 
 * From the above, 12 to 14 cycles interrupt latency should be the expected range.
 * When clock-cycle delay is measured from the start of the interrupt routine all code paths
 * do seem to have a constant delay, until the first time TIM1->CNT is read.
 * 
 * By analyzing all this the best I have seen is to not check the timer at all
 * when entering the interrupt service routine, and to precisely tune the 
 * start position to just when all address lines can be read valid when there is
 * a 12-cycle interrupt latency (the common case, and the lowest value)
 * 
 * To measure each path delay, a version of this code was used that stores
 * the timer value at the end of every code path in a fixed memory region,
 * then observed with ST-LINK debug tool.
 * 
 * TIM1->CNT access is slow, so it is not done until after the most critical
 * part of the code is complete. In practice the delay should be constant due to the
 * alignment on clock/4 grid happening along the way. 
 * 
 * gcc arm-none-eabi 9.2.1 exhibits some peculiar unoptimal code which some of has been worked around.
 * - Compiling with -O2 and -O3 produces *slower* code than compiling with -Os for this function.
 * - Compiling with -Os will ignore most standard "inline" function attribute.
 *   To still make gcc reliably inline a function while compiling with -Os use inline __attribute((always_inline))
 * - uint8_t and uint32_t: gcc generates ldrb instructions followed by utxb. This is not necessary.
 *   Where possible I've tried to force gcc to emit either ldrb/strb or ldr/str, using direct uint8_t casts.
 * - The first path after the if() is the branch-not-taken path. This has a fixed 1 cycle delay in ARM.
 *   I have not noticed any exceptions while checking the assembly.
 * - An if() statement to assign a different value to a variable is compiled usually with a branch.
 *   But the same statement using ? : seems to give the optimizer more freedom,
 *   can be compiled to a faster, time-constant itet eq - moveq - movne
 * - C switch-case statements are nice because the compiler can optimize the code with jump table.
 *   It turns out from tests that a jump-table (ARM tbb/tbh instruction) is 
 *   very stable in delay but is always much slower (6+ cycles) for the critical first path.
 *   Also the compiler re-orders case statements in any way it likes within the switch, causing the delays of each case statement to
 *   vary significantly from compile to compile after unrelated small code changes.
 *   Also this version of gcc adds a limit check before a jump table - even when the case table is complete.
 *   So all code below uses if() statements instead of switch-case, ordered by timing criticality,
 * - The optimizer will remove any variables it thinks are never used. This can be very tricky with the interrupt service routine.
 *   Some variables may need to be marked as "volatile" just to avoid the optimizer from removing it.
 * - Using a struct for the variables used by the interrupt service routine
 *   allows the compiler to have a single offset pointer from the linker.
 *   That saves a few cycles.
 * - Using an enum for hiram_state produces slower code. 
 *   An enum seems to block the compiler from doing many optimizations, 
 *   and it also produces unnecessary uint8_t / uint32_t casts.
 * 
 * Checking the KERNAL_CHECK timing on an oscilloscope:
 * 
 * - To check the timing details for the critical path, you can use the following trick:
 * - In the KERNAL_CHECK path, make both next hiram_states go to KERNAL_OFF
 * - In the $0001 check, remove the check for KERNAL_OFF. As a result the 
 *   KERNAL_CHECK is fired every time $01 is written but has just one cycle effect.
 * - Monitor the ROMH and A14 signal on the scope, or ROMH and D0, 
 *   or ROMH and PHI2, or ROMH and EXROM, or ROMH and GAME.
 * - Start an alternative or original bin kernal file.
 * - The kernal writes to $01 on every 50Hz interrupt. You can speed this up with a poke 1,55 basic loop
 * 
 *   In my test A14 reaches around 0.3-0.4V for 40ns when forced low. ROMH is low for 50ns.
 *   In skoe's doc above, it is mentioned A14 should go low not before 280ns after Phi2 high. 
 *   The rising edge of Phi2 is very slow so its hard to say exactly where it starts.
 *   The eye pattern of the data bit can be quite late and still be read properly by the 6510.
 *   Around 50ns before the falling edge of phi2 is still very stable. 
 *   The timing in skoe's doc shows 50ns, so does the KungFuFlash cartridge for other
 *   cartridge interrupt handlers.
 * 
 */

#define KERNAL_RAM   (0)
#define KERNAL_ROM   (1)
#define KERNAL_CHECK (2)
#define KERNAL_OFF   (3)

static struct {
	uint32_t hiram_state;
	uint32_t control_state;
	uint32_t otyper_fast;
} kernal_state;

// Defining these here again without DMB to save a few cycles.
void inline __attribute__((always_inline)) set_data_output(void) {
	*((volatile uint16_t *)&GPIO_DATABUS->MODER + GPIO_DATABUS_LANE) = 0x5555;
}

void inline __attribute__((always_inline)) set_data_input(void) {
	*((volatile uint16_t *)&GPIO_DATABUS->MODER + GPIO_DATABUS_LANE) = 0;
}

// uint32_t should be faster than uint8_t, but it isn't. Compiler generates spurious instructions
void inline __attribute__((always_inline)) set_data(uint8_t data) {
	*((volatile uint8_t *)&GPIO_DATABUS->ODR + GPIO_DATABUS_LANE) = data;
}

// uint32_t should be faster than uint8_t, but it isn't. Compiler generates spurious instructions
uint8_t inline __attribute__((always_inline)) get_data(void) {
    return (*((volatile uint8_t *)&GPIO_DATABUS->IDR + GPIO_DATABUS_LANE));
}

// Table ef_mode reworked to be a little faster using de02 lower bits.
static uint32_t const kernal_mode[8] = {
	CRT_PORT_ULTIMAX,
	CRT_PORT_ULTIMAX,
	CRT_PORT_16K,
	CRT_PORT_16K,
	
    CRT_PORT_NONE,
    CRT_PORT_ULTIMAX,
    CRT_PORT_8K,
    CRT_PORT_16K
};

/*************************************************
* C64 kernal interrupt handler
*************************************************/


static inline void kernal_handler() { 
	TIM1->SR = ~TIM_SR_CC3IF; // Clear timer irq immediately
	uint32_t addr;
	uint32_t control;
	uint32_t hiram_state= kernal_state.hiram_state;
	if(hiram_state == KERNAL_CHECK) {
		// Entry time is adjusted from the ISR timing. Address is read about 7 cycles earlier than control.
		COMPILER_BARRIER();
		addr = c64_addr_read(); // Address and BA/WRITE are stable here, but the PLA might not have decoded ROMH/ROML/IO1/IO2
		if ((addr & 0xe000) == 0xe000) {
			control = c64_control_read(); // PLA should have decoded everything here.
			uint32_t otyper=GPIO_OTHER->OTYPER; // Code runs slightly faster when this is moved before the next if()
			if ((control & (C64_BA | C64_WRITE)) == (C64_BA | C64_WRITE)) {
				// See skoe's excellent kernal document for details:
				c64_crt_control(C64_EXROM_LOW | C64_GAME_LOW);
				GPIO_OTHER->OTYPER = otyper & ~((1UL<<C64_GAME_POS) | (1UL<<C64_EXROM_POS)); // bic, str 2 cycle
				
				GPIO_ADDRESSBUS->MODER = (0x1UL << 14*2); // force A14 zero by setting A14 as output
				// A14 low-high should have effect around 40ns on ROMH. 4 cycles roughly delay needed
				// Prepare output type of GAME, EXROM as normal for rising flank below
				uint8_t data = ((uint8_t *)CRT_KERNAL)[addr & 0x1fff]; // 4 cycles
				GPIO_ADDRESSBUS->MODER = 0; // Release A14, set all address pins input
				uint32_t control2 = c64_control_read(); // ROMH changes slow enough to allow this to happen after releasing A14
				set_data(data); // Code runs slightly faster if done here.
				if((control2 & C64_ROMH)==0) {
					// ROMH is low, Kernal access. Set EXROM high fast. 
					c64_crt_control(C64_EXROM_HIGH | C64_GAME_LOW);
					kernal_state.hiram_state = KERNAL_ROM; // Need to wait a little otherwise there is data bus contention.
					set_data_output(); // Done here to 
					// Make EXROM open-drain here, should happen 4 cycles after setting EXROM high
					GPIO_OTHER->OTYPER=otyper & ~(1UL<<C64_GAME_POS);
					/* We releases the bus as fast as possible when phi2 is low */
					while(TIM1->CNT<150);
					c64_crt_control(kernal_state.control_state); // Back to the non-kernal access mode.
					set_data_input(); // 2 cycles
					__NOP(); // 1 extra cycle delay 
					GPIO_OTHER->OTYPER=otyper; // Set GAME open-drain, should happen 4 cycles after setting GAME HIGH.
					return;
				}
				// ROMH is high, DRAM access, release EXROM/GAME fast.
				c64_crt_control(C64_EXROM_HIGH | C64_GAME_HIGH); // Release EXROM/GAME for DRAM access
				kernal_state.hiram_state = KERNAL_RAM; // Should be 2 cycles
				__NOP(); // Wait 1 more cycle
				GPIO_OTHER->OTYPER=otyper; // Back to open-drain mode for EXROM and GAME, should happen 4 cycles after setting EXROM/GAME HIGH
				return;
			}
			return;
		}
	} else if(hiram_state == KERNAL_ROM) {
		COMPILER_BARRIER();
		addr = c64_addr_read();
		if ((addr & 0xe000) == 0xe000) {
			control = c64_control_read(); // ROML/ROMH should be stable here.
			uint32_t otyper=GPIO_OTHER->OTYPER; // slightly faster when done here.
			if ((control & (C64_BA | C64_WRITE)) == (C64_BA | C64_WRITE)) {
				uint32_t otyperx=otyper & kernal_state.otyper_fast;
				COMPILER_BARRIER();
				c64_crt_control(C64_EXROM_HIGH | C64_GAME_LOW); // Results in ROMH low pulse of 200ns
				GPIO_OTHER->OTYPER = otyperx; // only if exrom was low.
				uint8_t data = ((uint8_t *)CRT_KERNAL)[addr & 0x1fff];
				set_data(data);
				set_data_output(); // Should happen 40ns after ROMH low to avoid bus contention
				// Set GAME output type to normal for better rising edge flank. Has effect when GAME is set high below
				GPIO_OTHER->OTYPER = otyper & ~((1UL<<C64_GAME_POS));
				/* We releases the bus as fast as possible when phi2 is low */
				while(TIM1->CNT<150);
				c64_crt_control(kernal_state.control_state); // Back to normal access, 2 cycles
				set_data_input(); // 2 cycles, switch to input, lines will briefly float
				// 0 nops -> GAME takes 50ns to reach 1.0V
				// 1 or more nops -> GAME goes to 3v3 in about 15 ns. 
				// This is observed, but it is rather peculiar that a 1 cycle difference has so much effect.
				// It should be just 6ns difference for one nop. It could be alignment related.
				__NOP();
				GPIO_OTHER->OTYPER = otyper; // Set GAME back to open-drain, to let it rise to 5V.
				return;
			}
			return;
		}
	} else {
		COMPILER_BARRIER();
		addr = c64_addr_read();
		if ((addr & 0xe000) == 0x4000) { 
			// Added to try and make timing similar to above path. 
			// Using similar sized constants. Might need to be removed.
			// 0xe000 is used for ultimax so can't be kept same.
			return;
		}
	}
		
	control=c64_control_read();
	if((control & (C64_BA | C64_WRITE)) == (C64_BA | C64_WRITE)) {
		if((control & (C64_ROML|C64_ROMH)) != (C64_ROML|C64_ROMH)) {
			set_data(crt_ptr[addr & 0x3fff]);
			set_data_output();
		} else if((control & C64_IO1) == 0) {
			if((addr & 0xff) == 0x09) {
				set_data(*((uint8_t  *)&ef3_usb_rx_rdy) | *((uint8_t  *)&ef3_usb_tx_rdy));
				set_data_output();
			} else if((addr & 0xff) == 0x0a) {
				set_data(*((uint8_t  *)&ef3_usb_rx_data));
				set_data_output();
				ef3_usb_rx_rdy = EF3_NOT_RDY;
			}
		} else if((control & C64_IO2) == 0) {
			set_data(crt_ram_buf[addr & 0xff]);
			set_data_output();
		} else {
			return;
		}
		// wait for phi2 low and release. If we are slightly too fast in releasing doesn't matter.
		while(TIM1->CNT<150);
		set_data_input();
		return;
	}
	if((control & C64_WRITE) == 0) {
		if(((addr & (~1))==0x0000) && (hiram_state != KERNAL_OFF)) {
			kernal_state.hiram_state=KERNAL_CHECK;
			return;
		}
		if((control & C64_IO1) == 0) {
			if((addr & 0xff) == 0x2) {
				// This path is almost as timing critical as the KERNAL_CHECK path - this path needs full optimization.
				while(TIM1->CNT<120);
				uint8_t data=get_data();
				uint32_t control_state=kernal_mode[data & 0x7];
				kernal_state.control_state = control_state;
				c64_crt_control(control_state);
				// A ? : seems to produce the fastest code.
				kernal_state.hiram_state = (control_state == CRT_PORT_ULTIMAX) ? KERNAL_OFF : KERNAL_CHECK;
				// This code is faster than a ? : version. 
				kernal_state.otyper_fast = ~(~control_state & C64_EXROM_HIGH);
				led_set(data & 0x80); // Toggling the LED may be too slow, can be disabled to save a few cycles.
			} else if((addr & 0xff) == 0x0) {
				while(TIM1->CNT<120);
				uint8_t data=get_data();
				crt_ptr = crt_banks[data & 0x3f];
			} else if((addr & 0xff) == 0xa) {
				while(TIM1->CNT<120);
				uint8_t data=get_data();
				ef3_usb_tx_data = data;
				ef3_usb_tx_rdy = EF3_NOT_RDY;
			}
		} else if((control & C64_IO2) == 0 ) {
			while(TIM1->CNT<120);
			uint8_t data=get_data();
			crt_ram_buf[addr & 0xff] = data;
		}
	}
}

// The kernal_init handler does a little more than the ef_init for more code reuse.
static void kernal_init(uint32_t control_state)
{
	kernal_state.control_state=control_state;
	c64_crt_control(control_state);
	led_on();
	GPIO_ADDRESSBUS->ODR = 0x0; // prepare outputs on address bus as zero.
	kernal_state.hiram_state=(control_state == CRT_PORT_ULTIMAX) ? KERNAL_OFF : KERNAL_ROM;
	kernal_state.otyper_fast = ~(~control_state & C64_EXROM_HIGH);
	C64_INSTALL_HANDLER(kernal_handler);
}
