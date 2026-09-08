#include "task_manager.hpp"
#include "UartPort.hpp"
#include "memory_map.h"
#include "pid_controller.hpp"
#include "pid_viewer.hpp"

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "usart.h"

void led_test_entry(void* arg) {
    (void)arg;
    for (;;) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
        osDelay(250);
    }
}

Task<&led_test_entry, 128 * 4> led_test_task("led_test", osPriorityNormal);


namespace {

const PidController::Config position_pid_config{
    .kp = 2.0f,
    .ki = 0.2f,
    .kd = 0.01f,
    .max_out = 100.0f,
    .integral_limit = 20.0f,
};

const PidController::Config speed_pid_config{
    .kp = 1.2f,
    .ki = 0.1f,
    .kd = 0.005f,
    .max_out = 50.0f,
    .integral_limit = 10.0f,
};

const PidController::Config current_pid_config{
    .kp = 0.8f,
    .ki = 0.05f,
    .kd = 0.0f,
    .max_out = 20.0f,
    .integral_limit = 5.0f,
};

constexpr std::size_t pid_viewer_rx_buffer_size = 64U;
constexpr std::size_t pid_viewer_tx_buffer_size = 128U;

uint8_t pid_viewer_rx_buffer[pid_viewer_rx_buffer_size]{};
DMA_BUFFER_ATTR uint8_t pid_viewer_tx_buffer[pid_viewer_tx_buffer_size]{};

UartPort pid_viewer_uart{
    &huart4,
    pid_viewer_rx_buffer,
    sizeof(pid_viewer_rx_buffer),
    pid_viewer_tx_buffer,
    sizeof(pid_viewer_tx_buffer),
};

void pid_viewer_transmit(const uint8_t *data, std::size_t length) {
    (void)pid_viewer_uart.writeDma(data, length);
}

} // namespace

PidController position_test_pid("position_test", position_pid_config);
PidController speed_test_pid("speed_test", speed_pid_config);
PidController current_test_pid("current_test", current_pid_config);
PidController test_pid_3("test_pid_3", current_pid_config);
PidController test_pid_4("test_pid_4", current_pid_config);
PidController test_pid_5("test_pid_5", current_pid_config);
PidController test_pid_6("test_pid_6", current_pid_config);
PidController test_pid_7("test_pid_7", current_pid_config);
PidController test_pid_8("test_pid_8", current_pid_config);
PidController test_pid_9("test_pid_9", current_pid_config);
PidController test_pid_10("test_pid_10", current_pid_config);
PidController test_pid_11("test_pid_11", current_pid_config);
PidController test_pid_12("test_pid_12", current_pid_config);

void pid_control_test_entry(void* arg) {
    (void)arg;

    float position_measure = 0.0f;
    float speed_measure = 0.0f;
    float current_measure = 0.0f;

    for (;;) {
        const uint32_t timestamp_ms = osKernelGetTickCount();
        const bool positive_half_cycle = ((timestamp_ms / 3000U) & 1U) == 0U;

        const float position_ref = positive_half_cycle ? 60.0f : -60.0f;
        const float speed_ref = positive_half_cycle ? 30.0f : -30.0f;
        const float current_ref = positive_half_cycle ? 8.0f : -8.0f;

        const float position_out = position_test_pid.calculate(position_measure, position_ref);
        const float speed_out = speed_test_pid.calculate(speed_measure, speed_ref);
        const float current_out = current_test_pid.calculate(current_measure, current_ref);

        position_measure += position_out * 0.001f;
        speed_measure += (speed_out - speed_measure) * 0.02f;
        current_measure += (current_out - current_measure) * 0.05f;

        osDelay(1U);
    }
}

Task<&pid_control_test_entry, 128 * 4> pid_control_test_task(
    "pid_control_test", osPriorityHigh);



void pid_viewer_test_entry(void* arg) {
    (void)arg;

    PidViewer::set_transmit_callback(pid_viewer_transmit);
    (void)pid_viewer_uart.startRxDmaIdle();

    uint32_t last_receive_ms = 0U;
    bool receive_timeout_armed = false;

    for (;;) {
        const uint32_t timestamp_ms = osKernelGetTickCount();
        UartPort::Packet packet{};
        while (pid_viewer_uart.Read(packet)) {
            PidViewer::receive(packet.data, packet.len);
            last_receive_ms = timestamp_ms;
            receive_timeout_armed = true;
        }

        if (receive_timeout_armed &&
            static_cast<uint32_t>(timestamp_ms - last_receive_ms) >= 20U) {
            PidViewer::receive_timeout();
            receive_timeout_armed = false;
        }

        PidViewer::poll(timestamp_ms);
        osDelay(10U);
    }
}

Task<&pid_viewer_test_entry, 512 * 4> pid_viewer_test_task(
    "pid_viewer_test", osPriorityLow);
