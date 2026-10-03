#include "wit.h"

// DMA接收缓冲区，增大以容纳更多数据包
uint8_t wit_dmaBuffer[64];

// WIT传感器数据全局变量
volatile WIT_Data_t g_wit_data;

// 调试计数器
volatile uint32_t wit_debug_rx_count = 0;
volatile uint32_t wit_debug_valid_packet_count = 0;

/**
 * @brief  WIT传感器初始化函数
 * @note   启动DMA循环接收，使能UART接收中断
 * @param  None
 * @retval None
 */
void WIT_Init(void)
{
    // 清零数据结构
    memset((void*)&g_wit_data, 0, sizeof(WIT_Data_t));
    
    // 启动DMA循环接收（配合DMA_CIRCULAR模式）
    HAL_UART_Receive_DMA(&huart1, wit_dmaBuffer, sizeof(wit_dmaBuffer));
    // 使能UART空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/**
 * @brief  WIT传感器UART中断处理函数
 * @note   处理UART空闲中断，解析传感器数据包
 * @param  None
 * @retval None
 */
void WIT_UART_IRQHandler(void)
{
    uint8_t check_sum = 0;
    uint8_t rx_size;
    static uint16_t last_dma_pos = 0;
    uint16_t current_dma_pos;
    
    // 检查是否为UART空闲中断
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        // 清除空闲中断标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        
        // 获取当前DMA位置
        current_dma_pos = sizeof(wit_dmaBuffer) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        
        // 计算接收到的数据长度
        if (current_dma_pos >= last_dma_pos) {
            rx_size = current_dma_pos - last_dma_pos;
        } else {
            // DMA缓冲区循环回卷
            rx_size = sizeof(wit_dmaBuffer) - last_dma_pos + current_dma_pos;
        }
        
        wit_debug_rx_count++; // 调试计数
        
        // 处理接收到的数据包
        uint16_t pos = last_dma_pos;
        while (rx_size >= 11) {
            // 检查数据包头
            if (wit_dmaBuffer[pos % sizeof(wit_dmaBuffer)] == 0x55) {
                check_sum = 0;
                // 计算校验和
                for (int i = 0; i < 10; i++) {
                    check_sum += wit_dmaBuffer[(pos + i) % sizeof(wit_dmaBuffer)];
                }
                
                // 验证校验和
                if (check_sum == wit_dmaBuffer[(pos + 10) % sizeof(wit_dmaBuffer)]) {
                    wit_debug_valid_packet_count++; // 调试计数
                    
                    uint8_t data_type = wit_dmaBuffer[(pos + 1) % sizeof(wit_dmaBuffer)];
                    
                    // 解析加速度数据包 (0x51)
                    if (data_type == 0x51) {
                        int16_t ax_raw = (int16_t)((wit_dmaBuffer[(pos + 3) % sizeof(wit_dmaBuffer)] << 8) | 
                                                 wit_dmaBuffer[(pos + 2) % sizeof(wit_dmaBuffer)]);
                        int16_t ay_raw = (int16_t)((wit_dmaBuffer[(pos + 5) % sizeof(wit_dmaBuffer)] << 8) | 
                                                 wit_dmaBuffer[(pos + 4) % sizeof(wit_dmaBuffer)]);
                        int16_t az_raw = (int16_t)((wit_dmaBuffer[(pos + 7) % sizeof(wit_dmaBuffer)] << 8) | 
                                                 wit_dmaBuffer[(pos + 6) % sizeof(wit_dmaBuffer)]);
                        int16_t temp_raw = (int16_t)((wit_dmaBuffer[(pos + 9) % sizeof(wit_dmaBuffer)] << 8) | 
                                                   wit_dmaBuffer[(pos + 8) % sizeof(wit_dmaBuffer)]);
                        
                        g_wit_data.ax = ax_raw / 32768.0f * 16.0f; // g
                        g_wit_data.ay = ay_raw / 32768.0f * 16.0f; // g
                        g_wit_data.az = az_raw / 32768.0f * 16.0f; // g
                        g_wit_data.temperature = temp_raw / 100.0f; // °C
                    }
                    // 解析角速度数据包 (0x52)
                    else if (data_type == 0x52) {
                        int16_t gx_raw = (int16_t)((wit_dmaBuffer[(pos + 3) % sizeof(wit_dmaBuffer)] << 8) | 
                                                 wit_dmaBuffer[(pos + 2) % sizeof(wit_dmaBuffer)]);
                        int16_t gy_raw = (int16_t)((wit_dmaBuffer[(pos + 5) % sizeof(wit_dmaBuffer)] << 8) | 
                                                 wit_dmaBuffer[(pos + 4) % sizeof(wit_dmaBuffer)]);
                        int16_t gz_raw = (int16_t)((wit_dmaBuffer[(pos + 7) % sizeof(wit_dmaBuffer)] << 8) | 
                                                 wit_dmaBuffer[(pos + 6) % sizeof(wit_dmaBuffer)]);
                        
                        g_wit_data.gx = gx_raw / 32768.0f * 2000.0f; // °/s
                        g_wit_data.gy = gy_raw / 32768.0f * 2000.0f; // °/s
                        g_wit_data.gz = gz_raw / 32768.0f * 2000.0f; // °/s
                    }
                    // 解析角度数据包 (0x53)
                    else if (data_type == 0x53) {
                        int16_t roll_raw  = (int16_t)((wit_dmaBuffer[(pos + 3) % sizeof(wit_dmaBuffer)] << 8) | 
                                                    wit_dmaBuffer[(pos + 2) % sizeof(wit_dmaBuffer)]);
                        int16_t pitch_raw = (int16_t)((wit_dmaBuffer[(pos + 5) % sizeof(wit_dmaBuffer)] << 8) | 
                                                    wit_dmaBuffer[(pos + 4) % sizeof(wit_dmaBuffer)]);
                        int16_t yaw_raw   = (int16_t)((wit_dmaBuffer[(pos + 7) % sizeof(wit_dmaBuffer)] << 8) | 
                                                    wit_dmaBuffer[(pos + 6) % sizeof(wit_dmaBuffer)]);
                        int16_t version_raw = (int16_t)((wit_dmaBuffer[(pos + 9) % sizeof(wit_dmaBuffer)] << 8) | 
                                                      wit_dmaBuffer[(pos + 8) % sizeof(wit_dmaBuffer)]);
                        
                        g_wit_data.roll  = roll_raw / 32768.0f * 180.0f; // °
                        g_wit_data.pitch = pitch_raw / 32768.0f * 180.0f; // °
                        g_wit_data.yaw   = yaw_raw / 32768.0f * 180.0f; // °
                        g_wit_data.version = version_raw;
                    }
                    
                    pos += 11;
                    rx_size -= 11;
                } else {
                    // 校验失败，跳过一个字节继续搜索
                    pos++;
                    rx_size--;
                }
            } else {
                // 包头不匹配，跳过一个字节继续搜索
                pos++;
                rx_size--;
            }
        }
        
        // 更新最后处理的DMA位置
        last_dma_pos = current_dma_pos;
    }
}

#if 0
/**
 * @brief  USART2中断服务程序
 * @param  None
 * @retval None
 */
void USART2_IRQHandler(void)
{
    // 先调用WIT传感器处理函数
    WIT_UART_IRQHandler();
    // 再调用HAL库标准处理函数
    HAL_UART_IRQHandler(&huart1);
}
#endif

/**
 * @brief  获取调试信息
 * @param  rx_count: 接收中断次数
 * @param  valid_packet_count: 有效数据包计数
 * @retval None
 */
void WIT_GetDebugInfo(uint32_t *rx_count, uint32_t *valid_packet_count)
{
    *rx_count = wit_debug_rx_count;
    *valid_packet_count = wit_debug_valid_packet_count;
}

/**
 * @brief  获取原始DMA缓冲区数据（用于调试）
 * @param  buffer: 输出缓冲区指针
 * @param  size: 缓冲区大小
 * @retval 实际拷贝的数据大小
 */
uint16_t WIT_GetRawBuffer(uint8_t *buffer, uint16_t size)
{
    uint16_t copy_size = (size > sizeof(wit_dmaBuffer)) ? sizeof(wit_dmaBuffer) : size;
    memcpy(buffer, wit_dmaBuffer, copy_size);
    return copy_size;
}

/**
 * @brief  【新增】发送命令将当前航向角清零
 * @note   向维特传感器发送Z轴角度归零的指令
 * @param  None
 * @retval None
 */
void WIT_ZeroYaw(void)
{
    // 定义命令字节数组
    uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; // 1. 解锁配置
    uint8_t zero_yaw_cmd[] = {0xFF, 0xAA, 0x01, 0x04, 0x00}; // 2. Z轴角度归零
    uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};   // 3. 保存设置

    // 1. 发送解锁命令
    HAL_UART_Transmit(&huart1, unlock_cmd, sizeof(unlock_cmd), 100);
    HAL_Delay(100); // 等待传感器处理

    // 2. 发送Z轴归零命令
    HAL_UART_Transmit(&huart1, zero_yaw_cmd, sizeof(zero_yaw_cmd), 100);
    HAL_Delay(100); // 等待传感器处理

    // 3. 发送保存命令
    HAL_UART_Transmit(&huart1, save_cmd, sizeof(save_cmd), 100);
    HAL_Delay(100); // 等待设置生效
}