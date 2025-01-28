#[allow(unused_imports)]
#[cfg(target_os = "none")]
use defmt::{debug, error, info, trace, warn};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
#[allow(unused_imports)]
#[cfg(not(target_os = "none"))]
use log::{debug, error, info, trace, warn};
use nalgebra::Vector3;

use crate::flash_writer::FlashWriter;
use crate::flight_sm::{FlightSM, FlightState};
use crate::platform::{Accelerometer, AccelerometerType, Barometer, BarometerType};
use crate::WrenState;

#[embassy_executor::task]
pub async fn flight_task(
    mut baro: BarometerType,
    mut acc: AccelerometerType,
    flash: &'static Mutex<ThreadModeRawMutex, FlashWriter>,
    wren: &'static Mutex<ThreadModeRawMutex, WrenState>,
) -> ! {
    // write multiple times to guarante a proper sync if the previous state is corrupt
    {
        let mut flash = flash.lock().await;
        for _ in 0..16 {
            flash.write_power_on().await;
        }
    }

    let mut flight_sm = FlightSM::new();

    let mut max_altitude = 0_f32;
    let mut cycle_count = 0_u32;
    let mut local_voltage = 0.0;
    let mut ticker = Ticker::every(Duration::from_hz(50));

    loop {
        ticker.next().await;
        let baro_reading = match baro.read_temp_and_pressure().await {
            Ok(val) => val,
            Err(_) => {
                error!("Could not get baro measurement");
                continue;
            }
        };

        let acceleration = match acc.read_acceleration().await {
            Ok(val) => Vector3::<f32>::new(val[0], val[1], val[2]),
            Err(_) => {
                error!("Could not get accel measurement");
                continue;
            }
        };
        cycle_count += 1;

        let old_state = flight_sm.get_state();
        flight_sm.update(baro_reading.pressure, acceleration);
        let new_state = flight_sm.get_state();

        if flight_sm.get_relative_altitude() > max_altitude {
            max_altitude = flight_sm.get_relative_altitude();
        }

        let write_every = match flight_sm.get_state() {
            FlightState::MaybeLaunched => 1,
            FlightState::Boost => 1,
            FlightState::Coast => 1,
            FlightState::Descent => 10,
            FlightState::Idle => 50,
        };

        let mut flash = flash.lock().await;
        if cycle_count % write_every == 0 {
            flash.write_time(Instant::now()).await;
            flash.write_telem(
                    flight_sm.get_relative_altitude(),
                    flight_sm.get_vertical_velocity(),
                    flight_sm.get_vertical_acceleration(),
                ).await;
            flash.write_voltage(local_voltage).await;

            info!(
                "telemetry {} {} {}",
                flight_sm.get_relative_altitude(),
                flight_sm.get_vertical_velocity(),
                flight_sm.get_vertical_acceleration()
            );
            info!("flash {}", flash.get_index());
        }

        #[cfg(feature = "store_all")]
        if flight_sm.get_state() != FlightState::Idle {
            info!("wrote debug");
            flash
                .write_debug(
                    acceleration[0],
                    acceleration[1],
                    acceleration[2],
                    baro_reading.pressure,
                    Instant::now(),
                )
                .await;
        }
        if old_state != new_state {
            warn!("state changed {:?}", new_state);
            flash.write_state(new_state).await;
        }
        {
            let mut wren = wren.lock().await;
            wren.acceleration = flight_sm.get_raw_acceleration();
            wren.altitude = flight_sm.get_absolute_altitude();
            local_voltage = wren.battery_voltage;
        }

        trace!(
            "debug {:?} {} {} {}",
            new_state,
            flight_sm.get_absolute_altitude(),
            flight_sm.get_vertical_velocity(),
            flight_sm.get_vertical_acceleration()
        );
        trace!(
            "debug raw {} {} {} {}",
            baro_reading.pressure,
            acceleration[0],
            acceleration[1],
            acceleration[2]
        );
        trace!("debug flash {}", flash.get_index());
    }
}
