use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use crate::time::Instant;
use crate::twi::TWI;
use core::time::Duration;

const MS5611_ADDR: u8 = 0b0111_0111;
const REG_READ: u8 = 0x0;
const REG_D1: u8 = 0x40;
const REG_D2: u8 = 0x50;
const REG_PROM: u8 = 0xA0;

#[allow(dead_code)]
enum OverSamplingRatio {
    Opt256,
    Opt512,
    Opt1024,
    Opt2048,
    Opt4096,
}

impl OverSamplingRatio {
    fn get_delay(&self) -> Duration {
        Duration::from_micros(match *self {
            OverSamplingRatio::Opt256 => 1000,
            OverSamplingRatio::Opt512 => 2000,
            OverSamplingRatio::Opt1024 => 3000,
            OverSamplingRatio::Opt2048 => 5000,
            OverSamplingRatio::Opt4096 => 10000,
        })
    }

    fn addr_modifier(&self) -> u8 {
        match *self {
            OverSamplingRatio::Opt256 => 0,
            OverSamplingRatio::Opt512 => 2,
            OverSamplingRatio::Opt1024 => 4,
            OverSamplingRatio::Opt2048 => 6,
            OverSamplingRatio::Opt4096 => 8,
        }
    }
}

enum Ms5611LoopState {
    Reset,
    ReadD1 { start_time: Instant },
    ReadD2 { start_time: Instant, d1: u32 },
}

struct Ms5611 {
    /// We store the values C1-C6 from the memory of the MS5611
    /// We need to use them for later calculations
    /// From datasheet, C1.
    pressure_sensitivity: u16,
    /// From datasheet, C2.
    pressure_offset: u16,
    /// From datasheet, C3.
    temp_coef_pressure_sensitivity: u16,
    /// From datasheet, C4.
    temp_coef_pressure_offset: u16,
    /// From datasheet, C5.
    temp_ref: u16,
    /// From datasheet, C6.
    temp_coef: u16,

    /// What should the oversampling ratio of the chip be?
    over_sampling_ratio: OverSamplingRatio,

    /// State of the QMs5611 chip
    loop_state: Ms5611LoopState,

    /// Pressure in 10^-5 bar
    most_recent_pressure: u32,

    /// Temperature in centi-degrees Celsius
    most_recent_temperature: i32,
}

static BAROMETER: Mutex<OnceCell<Ms5611>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize() {
    // Safety: The TWI mutex is not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };

    let mut prom = [0; 8];
    let mut data = [0u8; 2];
    for c in 0..8 {
        _ = twi.read(MS5611_ADDR, REG_PROM + 2 * c, &mut data);
        prom[c as usize] = u16::from_be_bytes(data);
    }

    BAROMETER.modify(|baro| {
        baro.initialize(Ms5611 {
            pressure_sensitivity: prom[1],
            pressure_offset: prom[2],
            temp_coef_pressure_sensitivity: prom[3],
            temp_coef_pressure_offset: prom[4],
            temp_ref: prom[5],
            temp_coef: prom[6],
            over_sampling_ratio: OverSamplingRatio::Opt4096,
            loop_state: Ms5611LoopState::Reset,
            most_recent_pressure: 0,
            most_recent_temperature: 0,
        });
    });
}

fn update() {
    // Safety: The TWI and BAROMETER mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let baro = unsafe { BAROMETER.no_critical_section_lock_mut() };

    let now = Instant::now();

    match baro.loop_state {
        Ms5611LoopState::Reset => {
            //We let the chip know we want to read D1.
            twi.write(
                MS5611_ADDR,
                REG_D1 + baro.over_sampling_ratio.addr_modifier(),
                &[],
            );

            //Then set loop state for next iteration
            baro.loop_state = Ms5611LoopState::ReadD1 {
                start_time: Instant::now(),
            };
        }
        Ms5611LoopState::ReadD1 { start_time } => {
            //If the chip has not had enough time to process, return
            if now - start_time < baro.over_sampling_ratio.get_delay() {
                return;
            }

            //Read D1
            let mut buf = [0u8; 4];
            _ = twi.read(MS5611_ADDR, REG_READ, &mut buf[1..4]);
            let d1 = u32::from_be_bytes(buf);

            //We let the chip know we want to read D2.
            twi.write(
                MS5611_ADDR,
                REG_D2 + baro.over_sampling_ratio.addr_modifier(),
                &[],
            );

            //Then set loop state for next iteration
            baro.loop_state = Ms5611LoopState::ReadD2 {
                start_time: now,
                d1,
            };
        }
        Ms5611LoopState::ReadD2 { start_time, d1 } => {
            //If the chip has not had enough time to process, return
            if now - start_time < baro.over_sampling_ratio.get_delay() {
                return;
            }

            //Read D2
            let mut buf = [0u8; 4];
            _ = twi.read(MS5611_ADDR, REG_READ, &mut buf[1..4]);
            let d1 = u64::from(d1);
            let d2 = u64::from(u32::from_be_bytes(buf));

            //Use D1 and D2 to find the new pressure and temperature
            //Calculated using the ms5611 reference manual
            let dt: i64 = (d2 as i64) - ((baro.temp_ref as i64) << 8);
            let offset: i64 = ((baro.pressure_offset as i64) << 16)
                + ((dt * (baro.temp_coef_pressure_offset as i64)) >> 7);
            let sens: i64 = ((baro.pressure_sensitivity as i64) << 15)
                + ((dt * (baro.temp_coef_pressure_sensitivity as i64)) >> 8);

            // Compensation for low temperature
            let temp: i64 = 2000 + ((dt * (baro.temp_coef as i64)) >> 23);
            let (t2, off2, sens2);
            if temp <= 2000 {
                t2 = dt.pow(2) >> 31;
                off2 = 5 * (temp - 2000).pow(2) / 2;
                sens2 = off2 / 2;
            } else {
                t2 = 0;
                off2 = 0;
                sens2 = 0;
            }
            let temp = temp - t2;
            let offset = offset - off2;
            let sens = sens - sens2;

            baro.most_recent_pressure = (((((d1 as i64) * sens) >> 21) - offset) >> 15) as u32;
            baro.most_recent_temperature = temp as i32;

            //Then set loop state for next iteration, and we can do the next iteration immediately
            baro.loop_state = Ms5611LoopState::Reset;
            update();
        }
    }
}

/// Returns pressure in 10^-5 bar.
/// This function will never block, instead it will return an old value if no new value is available.
pub fn read_pressure() -> u32 {
    update();

    // Safety: The BAROMETER mutexes is not accessed in an interrupt
    let baro = unsafe { BAROMETER.no_critical_section_lock_mut() };
    baro.most_recent_pressure
}

/// Returns temperature in centi-degrees celsius (for example: 20.00 °C becomes 2000)
/// This function will never block, instead it will return an old value if no new value is available.
pub fn read_temperature() -> i32 {
    update();

    // Safety: The BAROMETER mutexes is not accessed in an interrupt
    let baro = unsafe { BAROMETER.no_critical_section_lock_mut() };
    baro.most_recent_temperature
}
