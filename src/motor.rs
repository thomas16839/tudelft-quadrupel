use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use cortex_m::peripheral::NVIC;
use nrf51_pac::{interrupt, Interrupt, GPIOTE, PPI};

struct Motors {
    motor_values: [u16; 4],
    motor_max: u16,
    timer1: nrf51_pac::TIMER1,
    timer2: nrf51_pac::TIMER2,
}

const MOTOR_0_PIN: u8 = 21;
const MOTOR_1_PIN: u8 = 23;
const MOTOR_2_PIN: u8 = 25;
const MOTOR_3_PIN: u8 = 29;

static MOTORS: Mutex<OnceCell<Motors>> = Mutex::new(OnceCell::uninitialized());

/// This sets the maximum motor value that the motor driver will cap the motor values at
/// For safety reasons, this is set to 400 by default. You need permission from a TA before changing this!
///
/// If you want the drone to actually fly:
/// - For old drones (carbon fiber), 800 is a reasonable maximum.
/// - For new drones (aluminium frame), 1000 is a reasonable maximum.
pub fn set_motor_max(max: u16) {
    MOTORS.modify(|motors| motors.motor_max = max);
}

/// This gets the maximum motor value that the motor driver will cap the motor values at. This is 400 by default.
pub fn get_motor_max() -> u16 {
    MOTORS.modify(|motors| motors.motor_max)
}

/// Get the current motor values. This is an array of four values:
/// - 0: The front motor
/// - 1: The right motor
/// - 2: The back motor
/// - 3: The left motor
pub fn get_motors() -> [u16; 4] {
    MOTORS.modify(|motors| motors.motor_values)
}

/// Set the motor values. This will cap the motor values to the value set by `set_motor_max`. The motor values are an array of four values:
/// - 0: The front motor
/// - 1: The right motor
/// - 2: The back motor
/// - 3: The left motor
pub fn set_motors(val: [u16; 4]) {
    MOTORS.modify(|guard| {
        guard.motor_values = val.map(|v| v.min(guard.motor_max));
    });
}

#[allow(clippy::too_many_lines)]
pub(crate) fn initialize(
    timer1: nrf51_pac::TIMER1,
    timer2: nrf51_pac::TIMER2,
    nvic: &mut NVIC,
    ppi: &mut PPI,
    gpiote: &mut GPIOTE,
) {
    MOTORS.modify(|motors| {
        motors.initialize(Motors {
            motor_values: [0; 4],
            motor_max: 400,
            timer1,
            timer2,
        });

        // Configure GPIOTE. GPIOTE is stands for GPIO tasks and events.
        // Safety: Writing the motor pins to the register is the intended behaviour. The pins have been verified to be correct.
        gpiote.config[0].write(|w| unsafe {
            w.mode()
                .task()
                .psel()
                .bits(MOTOR_0_PIN)
                .polarity()
                .toggle()
                .outinit()
                .set_bit()
        });
        gpiote.config[1].write(|w| unsafe {
            w.mode()
                .task()
                .psel()
                .bits(MOTOR_1_PIN)
                .polarity()
                .toggle()
                .outinit()
                .set_bit()
        });
        gpiote.config[2].write(|w| unsafe {
            w.mode()
                .task()
                .psel()
                .bits(MOTOR_2_PIN)
                .polarity()
                .toggle()
                .outinit()
                .set_bit()
        });
        gpiote.config[3].write(|w| unsafe {
            w.mode()
                .task()
                .psel()
                .bits(MOTOR_3_PIN)
                .polarity()
                .toggle()
                .outinit()
                .set_bit()
        });

        // Configure timer 2
        motors
            .timer2
            .prescaler
            .write(|w| unsafe { w.prescaler().bits(1) }); //0.125us. Safety: Allowed range of values is 0-9
        motors.timer2.intenset.write(|w| w.compare3().set_bit());
        motors.timer2.cc[0].write(|w| unsafe { w.bits(1000) }); // Safety: Any time is allowed
        motors.timer2.cc[1].write(|w| unsafe { w.bits(1000) }); // Safety: Any time is allowed
        motors.timer2.cc[3].write(|w| unsafe { w.bits(2500) }); // Safety: Any time is allowed
        motors.timer2.shorts.write(|w| w.compare3_clear().set_bit());
        motors.timer2.tasks_clear.write(|w| unsafe { w.bits(1) }); // Safety: Writing 1 to a task-clear register is allowed.

        // Configure timer 1
        // Safety: Allowed range of values is 0-9
        motors
            .timer1
            .prescaler
            .write(|w| unsafe { w.prescaler().bits(1) }); //0.125us
        motors.timer1.intenset.write(|w| w.compare3().set_bit());
        motors.timer1.cc[0].write(|w| unsafe { w.bits(1000) }); // Safety: Any time is allowed
        motors.timer1.cc[1].write(|w| unsafe { w.bits(1000) }); // Safety: Any time is allowed
        motors.timer1.cc[3].write(|w| unsafe { w.bits(2500) }); // Safety: Any time is allowed
        motors.timer1.shorts.write(|w| w.compare3_clear().set_bit());
        motors.timer1.tasks_clear.write(|w| unsafe { w.bits(1) }); // Safety: Writing 1 to a task-clear register is allowed.

        // Start the timer tasks
        // Safety: Writing 1 to a task-start registers is allowed.
        motors.timer2.tasks_start.write(|w| unsafe { w.bits(1) });
        motors.timer1.tasks_start.write(|w| unsafe { w.bits(1) });

        // Link motor 0 - gpiote 0
        ppi.ch[0]
            .eep
            .write(|w| unsafe { w.bits(motors.timer1.events_compare[0].as_ptr() as u32) });
        ppi.ch[0]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[0].as_ptr() as u32) });
        ppi.ch[1]
            .eep
            .write(|w| unsafe { w.bits(motors.timer1.events_compare[3].as_ptr() as u32) });
        ppi.ch[1]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[0].as_ptr() as u32) });

        // Link motor 1 - gpiote 1
        ppi.ch[2]
            .eep
            .write(|w| unsafe { w.bits(motors.timer1.events_compare[1].as_ptr() as u32) });
        ppi.ch[2]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[1].as_ptr() as u32) });
        ppi.ch[3]
            .eep
            .write(|w| unsafe { w.bits(motors.timer1.events_compare[3].as_ptr() as u32) });
        ppi.ch[3]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[1].as_ptr() as u32) });

        // Link motor 2 - gpiote 2
        ppi.ch[4]
            .eep
            .write(|w| unsafe { w.bits(motors.timer2.events_compare[0].as_ptr() as u32) });
        ppi.ch[4]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[2].as_ptr() as u32) });
        ppi.ch[5]
            .eep
            .write(|w| unsafe { w.bits(motors.timer2.events_compare[3].as_ptr() as u32) });
        ppi.ch[5]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[2].as_ptr() as u32) });

        // Link motor 3 - gpiote 3
        ppi.ch[6]
            .eep
            .write(|w| unsafe { w.bits(motors.timer2.events_compare[1].as_ptr() as u32) });
        ppi.ch[6]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[3].as_ptr() as u32) });
        ppi.ch[7]
            .eep
            .write(|w| unsafe { w.bits(motors.timer2.events_compare[3].as_ptr() as u32) });
        ppi.ch[7]
            .tep
            .write(|w| unsafe { w.bits(gpiote.tasks_out[3].as_ptr() as u32) });

        // Set which channels of PPI are enabled
        ppi.chenset.write(|w| {
            w.ch0()
                .set_bit()
                .ch1()
                .set_bit()
                .ch2()
                .set_bit()
                .ch3()
                .set_bit()
                .ch4()
                .set_bit()
                .ch5()
                .set_bit()
                .ch6()
                .set_bit()
                .ch7()
                .set_bit()
        });

        // Configure timer interrupts
        // Safety: We are not using priority-based critical sections.
        unsafe {
            nvic.set_priority(Interrupt::TIMER2, 1);
            NVIC::unpend(Interrupt::TIMER2);
            nvic.set_priority(Interrupt::TIMER1, 1);
            NVIC::unpend(Interrupt::TIMER1);
        }

        // Enable interrupts
        // Safety: We are not using mask-based critical sections.
        unsafe {
            NVIC::unmask(Interrupt::TIMER2);
            NVIC::unmask(Interrupt::TIMER1);
        }
    });
}

#[interrupt]
unsafe fn TIMER2() {
    // Safety: interrupts are already turned off here, since we are inside an interrupt
    let motors = unsafe { MOTORS.no_critical_section_lock_mut() };
    if motors.timer2.events_compare[3].read().bits() != 0 {
        motors.timer2.events_compare[3].reset();
        //2500 * 0.125
        motors.timer2.tasks_capture[2].write(|w| w.bits(1));

        if motors.timer2.cc[2].read().bits() < 500 {
            // Safety: Any time is allowed
            motors.timer2.cc[0].write(|w| w.bits(u32::from(1000 + motors.motor_values[2])));
            motors.timer2.cc[1].write(|w| w.bits(u32::from(1000 + motors.motor_values[3])));
        }
    }
}

#[interrupt]
unsafe fn TIMER1() {
    // Safety: interrupts are already turned off here, since we are inside an interrupt
    let motors = unsafe { MOTORS.no_critical_section_lock_mut() };
    if motors.timer1.events_compare[3].read().bits() != 0 {
        motors.timer1.events_compare[3].reset();
        motors.timer1.tasks_capture[2].write(|w| w.bits(1));

        if motors.timer1.cc[2].read().bits() < 500 {
            // Safety: Any time is allowed
            motors.timer1.cc[0].write(|w| w.bits(u32::from(1000 + motors.motor_values[0])));
            motors.timer1.cc[1].write(|w| w.bits(u32::from(1000 + motors.motor_values[1])));
        }
    }
}
