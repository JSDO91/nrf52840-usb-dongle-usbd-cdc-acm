#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf_drv_systick.h"
#include "nrf_delay.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrf_drv_timer.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "app_timer.h"
#include "app_error.h"


#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

#define BTN_CDC_DATA_SEND       0
#define BTN_CDC_NOTIFY_SEND     1

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
//static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
// 2020/06/21 jaesun, edit
static bool m_send_flag = false;
bool m_send_tx_done = false;

// 2022/06/20 jaesun
#define MAX_GTS_GAP     1024
#define MAX_USB_TX_BUF_LEN  240

static char m_tx_buffer[MAX_USB_TX_BUF_LEN] = {0,};
uint8_t usb_tx_buff[MAX_USB_TX_BUF_LEN]     = {0,};

static char m_rx_test_buffer[MAX_USB_TX_BUF_LEN];
static uint32_t m_rx_test_buffer_idx = 0;


uint32_t gts_start = 0;
uint32_t gts_end   = 0;
uint32_t gts_elapsed = 0;
uint32_t gts_gap[MAX_GTS_GAP] = {0,};
uint32_t gts_pre   = 0;
uint32_t gts_cur   = 0;
uint32_t gts_cnt   = 0;
uint32_t gtest_cnt = 0;
uint32_t gintr_cnt = 0;

uint32_t gtx_pkt_num = 0;
uint32_t gtx_save    = 0;
uint32_t tickvalue   = 0;
uint32_t gtick_1ms   = 0;

uint32_t totalTxByte = 0;

bool usb_open   = false;
bool usb_close  = false;

#define RECV_BUF_LEN 256

uint32_t gsingle_transfer_size = 0;
uint8_t gbuf_usb_rx[MAX_USB_TX_BUF_LEN]; 

static char m_cdc_data_array[RECV_BUF_LEN]  = {0};
volatile uint32_t buffer_available_widx      = 0;
volatile uint32_t buffer_available_ridx      = 0;


APP_TIMER_DEF(m_bsp_tmr);


uint32_t gDataGoodCnt = 0;
uint32_t gDataBadCnt  = 0;

void Check_Data(uint8_t* ptrbuf)
{
    uint8_t idx     = 0;
    uint8_t initval = ptrbuf[0];
    for(idx = 0; idx < MAX_USB_TX_BUF_LEN; idx++)
    {
        if(ptrbuf[idx] != ((initval+idx)&0xFF))
            gDataBadCnt++;
        else
            gDataGoodCnt++;

    }
}
////////////////////
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            app_usbd_cdc_acm_read(&m_app_cdc_acm, &m_cdc_data_array[buffer_available_widx++], READ_SIZE);  // necessary it seems
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            usb_close = true;
            usb_open  = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
// 2022/06/20 jaesun, add
            m_send_tx_done = true;            
            bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;

            do
            {
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, &m_cdc_data_array[buffer_available_widx], READ_SIZE);
                gbuf_usb_rx[gsingle_transfer_size++] = m_cdc_data_array[buffer_available_ridx];

                buffer_available_widx = (++buffer_available_widx)&0xFF;
                buffer_available_ridx = (++buffer_available_ridx)&0xFF;
            }
            while (ret == NRF_SUCCESS);
            
            if(gsingle_transfer_size == MAX_USB_TX_BUF_LEN)
            {
                Check_Data(gbuf_usb_rx);
                gsingle_transfer_size = 0;
            }
            m_send_flag     = true;
            m_send_tx_done  = true;

            bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
//            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
//            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
//            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}
/// end for USB CDC


/*****************************************************************************
 * FUNCTION NAME :  bsp_timer_handler
 * DESCRIPTION   :  
 * PARAMETER     :
 * RETURN VALUE  :
 *****************************************************************************/
static void bsp_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

#if 1
    if(gtx_save == 1)
    {
        tickvalue++;
        if(tickvalue >= 1000)
        {
            gts_pre             = gts_cur;
            gts_cur             = app_timer_cnt_get();
            gts_gap[gts_cnt++]  = gts_cur - gts_pre;
            __BKPT();
        }
    }
#else
    //gts_pre             = gts_cur;
    //gts_cur             = app_timer_cnt_get();
    //gts_gap[gts_cnt++]  = gts_cur - gts_pre;

    //if(gts_cnt >= MAX_GTS_GAP)
    //  gts_cnt = 0;

#endif
}

/*****************************************************************************
 * FUNCTION NAME :  timers_init
 * DESCRIPTION   :  Function for initializing the timer module.
 * PARAMETER     :
 * RETURN VALUE  :
 *****************************************************************************/
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    app_timer_create(&m_bsp_tmr, APP_TIMER_MODE_REPEATED, bsp_timer_handler);

    gtick_1ms = APP_TIMER_TICKS(1);
    app_timer_start(m_bsp_tmr, gtick_1ms, NULL);
}

void Make_USBD_TX_DATA_PATTERN(uint8_t* dataPtr, uint32_t dataLen, uint32_t dataTxCnt)
{
    uint8_t init  = (uint8_t)(dataTxCnt&0xFF);
    uint8_t idx   = 0;

    for(idx = 0; idx < 240; idx++)
    {
        dataPtr[idx] = (uint8_t)((idx + init)&0xFF);
    }
}



int main(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = 
    {
        .ev_state_proc = usbd_user_ev_handler
    };
    bsp_board_init(BSP_INIT_LEDS);
    ///start clock set mandatory for USB CDC
    nrf_drv_clock_init(); // for HF 32MHz external X-tal
    nrf_drv_clock_lfclk_request(NULL); // for LF 32.768kHz external X-tal
    while(!nrf_drv_clock_lfclk_is_running()) 
    {
        // Just waiting 
    }
    /// end clock set
    app_usbd_serial_num_generate();
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION) 
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
    }
  // 2022/06/20 jaesun,
    timers_init();

    while(true)
    {
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }


        Make_USBD_TX_DATA_PATTERN(&usb_tx_buff[0], MAX_USB_TX_BUF_LEN,gtx_pkt_num);

        if(m_send_flag && m_send_tx_done)
        {

#if 1
            if(gtx_save == 0)
            {
                gts_cur  = app_timer_cnt_get();
                gtx_save = 1;
            }
#else
            gts_pre             = gts_cur;
            gts_cur             = app_timer_cnt_get();
            gts_gap[gts_cnt++]  = gts_cur - gts_pre;

            if(gts_cnt >= MAX_GTS_GAP)
                gts_cnt = 0;
#endif
            memcpy(m_tx_buffer,usb_tx_buff,MAX_USB_TX_BUF_LEN);
            app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, MAX_USB_TX_BUF_LEN);

            totalTxByte     += MAX_USB_TX_BUF_LEN;
            gtx_pkt_num++;
            m_send_tx_done  = false;
        }
    }
}
