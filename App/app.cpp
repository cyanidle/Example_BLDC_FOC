#include "app.h"

#include <memory>

#include "tim.h"
#include "i2c.h"
#include "adc.h"
#include "fdcan.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <uavcan/primitive/scalar/Natural32_1_0.h>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/hall_sensor/hall_sensor.h>
#include <voltbro/motors/bldc/six_step/six_step_controller.h>


EEPROM eeprom(&hi2c4);

HallSensor hall_sensor(
    12,
    false,
    ENC_1_GPIO_Port,
    ENC_1_Pin,
    ENC_3_GPIO_Port,
    ENC_3_Pin,
    ENC_2_GPIO_Port,
    ENC_2_Pin,
    {
        HallPhase::PHASE_B,
        HallPhase::PHASE_C,
        HallPhase::PHASE_A
    }
);

std::unique_ptr<SixStepController> motor;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENC_1_Pin || GPIO_Pin == ENC_2_Pin || GPIO_Pin == ENC_3_Pin) {
        hall_sensor.handle_hall_channel(GPIO_Pin);
    }
}

static volatile encoder_data enc_val = 0;
static volatile int enc_rev = 0;
[[noreturn]] void app() {
    start_timers();
    start_cyphal();

    while (!eeprom.is_connected()) {
        eeprom.delay();
    }
    eeprom.delay();

    motor = std::make_unique<SixStepController>(
        0,
        DriveInfo {
            .torque_const = 0.069,  // Nm / A == V / (rad/s)
            .speed_const = 24.42,   // (rad/s) / V
            .max_current = 22,
            .max_torque = 1,
            .stall_current = 13,
            .stall_timeout = 3,
            .stall_tolerance = 0.2,
            .supply_voltage = 24,
            .l_pins = {
                GpioPin(INLA_GPIO_Port, INLA_Pin),
                GpioPin(INLB_GPIO_Port, INLB_Pin),
                GpioPin(INLC_GPIO_Port, INLC_Pin)
            },
            .en_pin = GpioPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin),
            .common = {
                .ppairs = 2,
                .gear_ratio = 1
            }
        },
        &htim1,
        &hadc1,
        hall_sensor
    );

    HAL_IMPORTANT(motor->init())
    HAL_IMPORTANT(motor->start())

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    while(true) {
        motor->update();
        cyphal_loop();
    }
}

TYPE_ALIAS(Natural32, uavcan_primitive_scalar_Natural32_1_0)
TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
static constexpr CanardPortID ENCODER_PORT = 7100;
static constexpr CanardPortID VOLTAGE_PORT = 5800;

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    static const auto node_id = get_node_id();
    if ((current_t - report_time) >= (50)) {
        Natural32 ::Type enc_msg = {};
        enc_msg.value = hall_sensor.get_value();
        static CanardTransferID enc_transfer_id = 0;
        get_interface()->send_msg<Natural32>(&enc_msg, ENCODER_PORT + node_id, &enc_transfer_id);
        report_time = current_t;
    }
}

class VoltageSub: public AbstractSubscription<Real32> {
public:
VoltageSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32::Type& msg, CanardRxTransfer* _) override {
        motor->set_voltage_point(msg.value);
    }
};

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<1>> registers_handler;
ReservedObject<VoltageSub> voltage_sub;

void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    auto cyphal_interface = get_interface();
    const auto node_id = get_node_id();

    voltage_sub.create(cyphal_interface, VOLTAGE_PORT + node_id);
    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.bldc_6step_example",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );
    registers_handler.create(
        std::array<RegisterDefinition, 1>{{
            {
                "motor.is_on",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    static bool value = false;
                    if (v_in._tag_ == 3) {
                        value = v_in.bit.value.bitpacked[0] == 1;
                    }
                    else {
                        // TODO: report error
                    }

                    motor->set_state(value);

                    response.persistent = true;
                    response._mutable = true;
                    v_out._tag_ = 3;
                    v_out.bit.value.bitpacked[0] = motor->is_on();
                    v_out.bit.value.count = 1;
                }
            }
        }},
        cyphal_interface
    );

    static FDCAN_FilterTypeDef sFilterConfig;
    uint32_t filter_index = 0;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        node_info_reader->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        registers_handler->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        voltage_sub->make_filter(node_id)
    ))
}
