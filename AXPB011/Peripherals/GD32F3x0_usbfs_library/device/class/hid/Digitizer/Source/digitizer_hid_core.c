/*******************************************************************************
*                                    NOTICE
*
* Copyright (c) 2010 - 2022 TouchNetix Limited
* ALL RIGHTS RESERVED.
*
* The source  code contained  or described herein  and all documents  related to
* the source code ("Material") are owned by TouchNetix Limited ("TouchNetix") or
* its suppliers  or licensors. Title to the Material  remains with TouchNetix or
* its   suppliers  and  licensors. The  Material  contains  trade  secrets   and
* proprietary  and confidential information  of TouchNetix or its  suppliers and
* licensors.  The  Material  is  protected  by  worldwide  copyright  and  trade
* secret  laws and  treaty  provisions.  No part  of the Material  may be  used,
* copied,  reproduced,  modified,   published,  uploaded,  posted,  transmitted,
* distributed or disclosed in any way without TouchNetix's prior express written
* permission.
* 
* No  license under any  patent, copyright,  trade secret or other  intellectual
* property  right is granted to or conferred upon you by disclosure or  delivery
* of  the Materials, either  expressly, by implication, inducement, estoppel  or
* otherwise.  Any  license  under  such  intellectual  property rights  must  be
* expressly approved by TouchNetix in writing.
*
*******************************************************************************/

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "digitizer_hid_core.h"
#include "usbd_enum.h"
#include "mode_control.h"
#include "timers_and_leds.h"
#include <string.h>

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/


/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define USBD_VID                     0x28E9U
#define USBD_PID                     0x028AU

/* USB Report Descriptor - Absolute mouse mode */
uint8_t mouse_abs_report_descriptor[ABS_MOUSE_REPORT_DESC_LEN] =
{
    0x05, 0x01,         // Usage Page (Generic Desktop),
    0x09, 0x02,         // Usage (Mouse),
    0xA1, 0x01,         // Collection (Application),
    0x09, 0x01,         //   Usage (Pointer),
    0xA1, 0x00,         //   Collection (Physical),
    0x05, 0x09,         //     Usage Page (Buttons),
    0x19, 0x01,         //     Usage Minimum (01),
    0x29, 0x03,         //     Usage Maximum (03),
    0x15, 0x00,         //     Logical Minimum (0),
    0x25, 0x01,         //     Logical Maximum (1),
    0x75, 0x01,         //     Report Size (1),
    0x95, 0x03,         //     Report Count (3),
    0x81, 0x02,         //     Input (Data, Variable, Absolute)
    0x75, 0x05,         //     Report Size (5),
    0x95, 0x01,         //     Report Count (1),
    0x81, 0x01,         //     Input (Constant),
    0x05, 0x01,         //     Usage Page (Generic Desktop),
    0x16, 0x00, 0x00,   //     Logical Minimum (0),     /* 35(low byte) 36(high byte) */
    0x26, 0xFF, 0x0F,   //     Logical Maximum (4095),  /* 41(low byte) 42(high byte) */
    0x36, 0x00, 0x00,   //     Physical Minimum (0),    /* 38(low byte) 39(high byte) */
    0x46, 0xFF, 0x0F,   //     Physical Maximum (4095), /* 44(low byte) 45(high byte) */
    0x09, 0x30,         //     Usage (X),
    0x09, 0x31,         //     Usage (Y),
    0x75, 0x10,         //     Report Size (16),
    0x95, 0x02,         //     Report Count (2),
    0x81, 0x02,         //     Input (Data, Variable, Absolute)
    0xC0,               //   End Collection,
    0xC0,               // End Collection
};

/* USB Report Descriptor - Parallel Digitizer mode */
uint8_t digitizer_report_descriptor[DIGITIZER_REPORT_DESC_LEN] =
{
    // Top Level Collection - Touchscreen Digitizer */
    0x05u, 0x0Du,                         // USAGE_PAGE (Digitizers) */
    0x09u, 0x04u,                         // USAGE (Touch Screen) */
    0xA1u, 0x01u,                         // COLLECTION (Application) */
    0x85u, 0x01,                          // REPORT_ID (Touch) */
    /* 7 */

    //bring all the common parts between touch reports out here to reduce the descriptor size (else end up repeating same settings many times)
    0x35u, 0x00u,                         //     PHYSICAL MINIMUM (0) */
    0x15u, 0x00u,                         //     LOGICAL MINIMUM (0) */
    0x55u, 0x0Eu,                         //     Unit exponent (-2) */
    0x65u, 0x11u,                         //     UNIT (cm) */
    /* 13 */

    // First contact report */
    0x05u, 0x0Du,                         // USAGE_PAGE (Digitizers) */
    0x09u, 0x22u,                         // USAGE (Finger) */
    0xA1u, 0x02u,                         //     COLLECTION (Logical) */
    0x25u, 0x01u,                         //     LOGICAL_MAXIMUM (1) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x75u, 0x01u,                         //     REPORT_SIZE (1) */

    0x09u, 0x42u,                         //     USAGE (Tip Switch) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = TipSwitch status (Touch / No Touch) */

    0x09u, 0x32u,                         //     USAGE (In Range) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bits = In range */

    0x09u, 0x47u,                         //     USAGE (Confidence) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = Confidence */

    0x25u, 0x1Fu,                         //     LOGICAL_MAXIMUM (31) */
    0x75u, 0x05u,                         //     REPORT SIZE (5) */
    0x09u, 0x51u,                         //     USAGE (CONTACT ID) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     5 Bits = Contact ID */

    0x05u, 0x01u,                         //     USAGE_PAGE (Generic Desktop) */
    0x75u, 0x10u,                         //     REPORT_SIZE (16) */
    /* 50 */

    /* NOTE - having to use 4 bytes for logical and physical values here:
    * if using 2 bytes to report a user requests the maximum possible value according to TH2 (0xFFFF) then we will end up with a negative value (as this is reported in 2s complement)
    * so need to stretch to being represented using 4 bytes to prevent this from happening (can't use 3 bytes, hence 4)
    * plus the physical value reported can go above 0xFFFF as we multiply the value read from aXiom by 5 to obtain the correct values
    * (TH2 increments by 0.5mm whilst in here we increment by 0.1mm)
    */
    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  X (default = 4095) */ /* 53(low byte), 56(high byte) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM X (default = 0) */    /* 58(low byte), 61(high byte) */
    0x09u, 0x30u,                         //     USAGE (X) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = X Position */
    /* 64 */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  Y (default = 4095) */ /* 67(low), 70(high) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM Y (default = 0) */    /* 72(low byte), 75(high byte) */
    0x09u, 0x31u,                         //     USAGE (Y) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = Y Position */
    /* 78 */

    /* Touch Pressure - used to convey z values */
    0x05u, 0x0Du,                         //     USAGE_PAGE (Digitizer) */
    0x09u, 0x30u,                         //     USAGE (Pressure) */
    0x26u, 0x00u, 0x04u,                  //     LOGICAL_MAXIMUM (1024) */
    0x45u, 0x00u,                         //     PHYSICAL_MAXIMUM (0) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits */

    0xC0u,                                // END_COLLECTION (Logical / 1st contact) */
    /* 93 */

    // Second Contact */
    0x05u, 0x0Du,                         // USAGE_PAGE (Digitizers) */
    0x09u, 0x22u,                         // USAGE (Finger) */
    0xA1u, 0x02u,                         //     COLLECTION (Logical) */
    0x25u, 0x01u,                         //     LOGICAL_MAXIMUM (1) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x75u, 0x01u,                         //     REPORT_SIZE (1) */

    0x09u, 0x42u,                         //     USAGE (Tip Switch) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = TipSwitch status (Touch / No Touch) */

    0x09u, 0x32u,                         //     USAGE (In Range) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bits = In range */

    0x09u, 0x47u,                         //     USAGE (Confidence) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = Confidence */

    0x25u, 0x1Fu,                         //     LOGICAL_MAXIMUM (31) */
    0x75u, 0x05u,                         //     REPORT SIZE (5) */
    0x09u, 0x51u,                         //     USAGE (CONTACT ID) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     5 Bits = Contact ID */

    0x05u, 0x01u,                         //     USAGE_PAGE (Generic Desktop) */
    0x75u, 0x10u,                         //     REPORT_SIZE (16) */
    /* 129 */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  X (default = 4095) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM X (default = 0) */
    0x09u, 0x30u,                         //     USAGE (X) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = X Position */
    /* 143 */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  Y (default = 4095) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM Y (default = 0) */
    0x09u, 0x31u,                         //     USAGE (Y) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = Y Position */
    /* 157 */

    /* Touch Pressure - used to convey z values */
    0x05u, 0x0Du,                         //     USAGE_PAGE (Digitizer) */
    0x09u, 0x30u,                         //     USAGE (Pressure) */
    0x26u, 0x00u, 0x04u,                  //     LOGICAL_MAXIMUM (1024) */
    0x45u, 0x00u,                         //     PHYSICAL_MAXIMUM (0) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits */

    0xC0u,                                // END_COLLECTION (Logical / 2nd contact) */
    /* 172 */

    // Third Contact */
    0x05u, 0x0Du,                         // USAGE_PAGE (Digitizers) */
    0x09u, 0x22u,                         // USAGE (Finger) */
    0xA1u, 0x02u,                         //     COLLECTION (Logical) */
    0x25u, 0x01u,                         //     LOGICAL_MAXIMUM (1) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x75u, 0x01u,                         //     REPORT_SIZE (1) */

    0x09u, 0x42u,                         //     USAGE (Tip Switch) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = TipSwitch status (Touch / No Touch) */

    0x09u, 0x32u,                         //     USAGE (In Range) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bits = In range */

    0x09u, 0x47u,                         //     USAGE (Confidence) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = Confidence */

    0x25u, 0x1Fu,                         //     LOGICAL_MAXIMUM (31) */
    0x75u, 0x05u,                         //     REPORT SIZE (5) */
    0x09u, 0x51u,                         //     USAGE (CONTACT ID) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     5 Bits = Contact ID */

    0x05u, 0x01u,                         //     USAGE_PAGE (Generic Desktop) */
    0x75u, 0x10u,                         //     REPORT_SIZE (16) */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  X (default = 4095) */ /* 163(low), 164(high) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM X (default = 0) */    /* 166(low), 167(high) */
    0x09u, 0x30u,                         //     USAGE (X) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = X Position */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  Y (default = 4095) */ /* 173(low), 174(high) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM Y (default = 0) */    /* 176(low), 177(high) */
    0x09u, 0x31u,                         //     USAGE (Y) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = Y Position */

    /* Touch Pressure - used to convey z values */
    0x05u, 0x0Du,                         //     USAGE_PAGE (Digitizer) */
    0x09u, 0x30u,                         //     USAGE (Pressure) */
    0x26u, 0x00u, 0x04u,                  //     LOGICAL_MAXIMUM (1024) */
    0x45u, 0x00u,                         //     PHYSICAL_MAXIMUM (0) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits */

    0xC0u,                                // END_COLLECTION (Logical / 3rd contact) */

    // Fourth Contact */
    0x05u, 0x0Du,                         // USAGE_PAGE (Digitizers) */
    0x09u, 0x22u,                         // USAGE (Finger) */
    0xA1u, 0x02u,                         //     COLLECTION (Logical) */
    0x25u, 0x01u,                         //     LOGICAL_MAXIMUM (1) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x75u, 0x01u,                         //     REPORT_SIZE (1) */

    0x09u, 0x42u,                         //     USAGE (Tip Switch) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = TipSwitch status (Touch / No Touch) */

    0x09u, 0x32u,                         //     USAGE (In Range) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bits = In range */

    0x09u, 0x47u,                         //     USAGE (Confidence) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = Confidence */

    0x25u, 0x1Fu,                         //     LOGICAL_MAXIMUM (31) */
    0x75u, 0x05u,                         //     REPORT SIZE (5) */
    0x09u, 0x51u,                         //     USAGE (CONTACT ID) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     5 Bits = Contact ID */

    0x05u, 0x01u,                         //     USAGE_PAGE (Generic Desktop) */
    0x75u, 0x10u,                         //     REPORT_SIZE (16) */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  X (default = 4095) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM X (default = 0) */
    0x09u, 0x30u,                         //     USAGE (X) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = X Position */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  Y (default = 4095) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM Y (default = 0) */
    0x09u, 0x31u,                         //     USAGE (Y) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = Y Position */

    /* Touch Pressure - used to convey z values */
    0x05u, 0x0Du,                         //     USAGE_PAGE (Digitizer) */
    0x09u, 0x30u,                         //     USAGE (Pressure) */
    0x26u, 0x00u, 0x04u,                  //     LOGICAL_MAXIMUM (1024) */
    0x45u, 0x00u,                         //     PHYSICAL_MAXIMUM (0) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits */

    0xC0u,                                // END_COLLECTION (Logical / 4th contact) */

    // Fifth Contact */
    0x05u, 0x0Du,                         // USAGE_PAGE (Digitizers) */
    0x09u, 0x22u,                         // USAGE (Finger) */
    0xA1u, 0x02u,                         //     COLLECTION (Logical) */
    0x25u, 0x01u,                         //     LOGICAL_MAXIMUM (1) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x75u, 0x01u,                         //     REPORT_SIZE (1) */

    0x09u, 0x42u,                         //     USAGE (Tip Switch) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = TipSwitch status (Touch / No Touch) */

    0x09u, 0x32u,                         //     USAGE (In Range) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bits = In range */

    0x09u, 0x47u,                         //     USAGE (Confidence) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     1 Bit = Confidence */

    0x25u, 0x1Fu,                         //     LOGICAL_MAXIMUM (31) */
    0x75u, 0x05u,                         //     REPORT SIZE (5) */
    0x09u, 0x51u,                         //     USAGE (CONTACT ID) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     5 Bits = Contact ID */

    0x05u, 0x01u,                         //     USAGE_PAGE (Generic Desktop) */
    0x75u, 0x10u,                         //     REPORT_SIZE (16) */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,     //     LOGICAL_MAXIMUM  X (default = 4095) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,     //     PHYSICAL_MAXIMUM X (default = 0) */
    0x09u, 0x30u,                         //     USAGE (X) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = X Position */

    0x27u, 0xFFu, 0x0Fu, 0x00u, 0x00u,    //     LOGICAL_MAXIMUM  Y (default = 4095) */
    0x47u, 0x00u, 0x00u, 0x00u, 0x00u,    //     PHYSICAL_MAXIMUM Y (default = 0) */
    0x09u, 0x31u,                         //     USAGE (Y) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits = Y Position */

    /* Touch Pressure - used to convey z values */
    0x05u, 0x0Du,                         //     USAGE_PAGE (Digitizer) */
    0x09u, 0x30u,                         //     USAGE (Pressure) */
    0x26u, 0x00u, 0x04u,                  //     LOGICAL_MAXIMUM (1024) */
    0x45u, 0x00u,                         //     PHYSICAL_MAXIMUM (0) */
    0x95u, 0x01u,                         //     REPORT_COUNT (1) */
    0x81u, 0x02u,                         //     INPUT (Data,Var,Abs)     16 Bits */

    0xC0u,                                // END_COLLECTION (Logical / 5th contact) */

    // Timestamp */
    0x05, 0x0d,                         //    USAGE_PAGE (Digitizers)
    0x55, 0x0C,                         //    UNIT_EXPONENT (-4)   !!!MUST be -4 i.e. 100us for Win8
    0x66, 0x01, 0x10,                   //    UNIT (Seconds)
    0x47, 0xff, 0xff, 0x00, 0x00,       //    PHYSICAL_MAXIMUM (65535)
    0x27, 0xff, 0xff, 0x00, 0x00,       //    LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                         //    REPORT_SIZE (16)
    0x95, 0x01,                         //    REPORT_COUNT (1)
    0x09, 0x56,                         //    USAGE (Scan Time)
    0x81, 0x02,                         //    INPUT (Data,Var,Abs)


    // Contact Count */
    0x05u, 0x0Du,                         // USAGE_PAGE (Digitizers) */
    0x15u, 0x00u,                         // LOGICAL_MINIMUM (0) */
    0x25u, 0x1Fu,                         // LOGICAL_MAXIMUM (31) */
    0x75u, 0x05u,                         // REPORT SIZE (5) */
    0x09u, 0x54u,                         // USAGE (Contact Count) */
    0x95u, 0x01u,                         // REPORT COUNT (1) */
    0x81u, 0x02u,                         // INPUT (Data,Var,Abs)     5 Bits = Contact count */

    0x75u, 0x03u,                         // REPORT_SIZE (3) */
    0x25u, 0x01u,                         // LOGICAL_MAXIMUM (1) */
    0x95u, 0x01u,                         // REPORT COUNT (1) */
    0x81u, 0x03u,                         // Input (Cnst,Var,Abs)     3 Bits = Padding */

    /* Feature report notification */
    0x85u, 0x02u,      /*   Report ID (Feature) */
    0x75u, 0x08u,                         /*   REPORT SIZE (8) */
    0x09u, 0x55u,                         /*   USAGE (Maximum Count) */
    0x25u, 0x0Au,                         /*   Logical maximum (10) */
    0xB1u, 0x02u,                         /*   Feature (Data, Var, Abs) */

    0xC0u,                               // END_COLLECTION */
};

/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/


/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/
#define UNUSED(x) ((void)(x))

/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static uint8_t digitizer_hid_init(usb_dev *udev, uint8_t config_index, usb_desc_ep *epin, usb_desc_ep *epout);
static uint8_t digitizer_hid_deinit(usb_dev *udev, uint8_t config_index);
static uint8_t digitizer_hid_req_handler(usb_dev *udev, usb_req *req);
static uint8_t digitizer_hid_data_in(usb_dev *udev, uint8_t ep_num);
static uint8_t digitizer_hid_data_out(usb_dev *udev, uint8_t ep_num);

usb_custom_class_core usbd_digitizer_hid_cb = {
    .command   = NO_CMD,
    .alter_set = 0U,

    .init      = digitizer_hid_init,
    .deinit    = digitizer_hid_deinit,

    .req_proc  = digitizer_hid_req_handler,

    .data_in   = digitizer_hid_data_in,
    .data_out  = digitizer_hid_data_out
};

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/*!
    \brief      register HID interface operation functions
    \param[in]  udev: pointer to USB device instance
    \param[in]  digitizer_fop_handler: HID operation functions structure
    \param[out] none
    \retval     USB device operation status
*/
uint8_t digitizer_hid_itfop_register(usb_dev *udev, digitizer_fop_handler *hid_fop)
{
    if (NULL != hid_fop)
    {
        udev->dev.user_data[DIGITIZER_INTERFACE_NUM] = hid_fop;

        return USBD_OK;
    }

    return USBD_FAIL;
}

/*!
    \brief      send digitizer HID report
    \param[in]  udev: pointer to USB device instance
    \param[in]  report: pointer to HID report
    \param[in]  len: data length
    \param[out] none
    \retval     USB device operation status
*/
uint8_t digitizer_hid_report_send(usb_dev *udev, uint8_t *report, uint32_t len)
{
    usbd_status status = USBD_BUSY;

    digitizer_hid_handler *hhid = (digitizer_hid_handler *)udev->dev.class_data[DIGITIZER_INTERFACE_NUM];

    if (udev->dev.cur_status == USBD_CONFIGURED)
    {
        if (hhid->state == eIDLE)
        {
            hhid->state = eBUSY;
            usbd_ep_send(udev, DIGITIZER_HID_IN_EP, report, len);
            NotifyUSBComms();
            status = USBD_OK;
        }
    }

    return status;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
/*!
    \brief      initialize the HID device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t digitizer_hid_init(usb_dev *udev, uint8_t config_index, usb_desc_ep *epin, usb_desc_ep *epout)
{
    UNUSED(epout);

    static digitizer_hid_handler hid_handler;

    memset((void *)&hid_handler, 0U, sizeof(digitizer_hid_handler));

    /* Initialize the data Tx endpoint */
    usbd_ep_setup(udev, epin);

    udev->dev.class_data[DIGITIZER_INTERFACE_NUM] = (void *)&hid_handler;

    if (udev->dev.user_data[DIGITIZER_INTERFACE_NUM] != NULL)
    {
        if (((digitizer_fop_handler*) udev->dev.user_data[DIGITIZER_INTERFACE_NUM])->periph_config != NULL)
        {
            ((digitizer_fop_handler*) udev->dev.user_data[DIGITIZER_INTERFACE_NUM])->periph_config();
        }
    }

    return USBD_OK;
}

/*!
    \brief      de-initialize the HID device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t digitizer_hid_deinit(usb_dev *udev, uint8_t config_index)
{
    /* deinitialize HID endpoints */
    usbd_ep_clear(udev, DIGITIZER_HID_IN_EP);

    return USBD_OK;
}

/*!
    \brief      handle the HID class-specific requests
    \param[in]  udev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t digitizer_hid_req_handler(usb_dev *udev, usb_req *req)
{
    usb_transc *transc = &udev->dev.transc_in[0];

    digitizer_hid_handler *hid = (digitizer_hid_handler *)udev->dev.class_data[DIGITIZER_INTERFACE_NUM];

    switch(req->bRequest) {
    case GET_REPORT:
        uint8_t buf[64] = {0};
        buf[0] = 0x02u;  // report ID
        buf[1] = 0x05u;  // max no. contacts

        transc->xfer_buf = (uint8_t *)buf;
        transc->remain_len = req->wLength;
        break;

    case GET_IDLE:
        transc->xfer_buf = (uint8_t *)&hid->idlestate;
        transc->remain_len = 1U;
        break;

    case GET_PROTOCOL:
        transc->xfer_buf = (uint8_t *)&hid->protocol;
        transc->remain_len = 1U;
        break;

    case SET_REPORT:
        hid->reportID = (uint8_t)(req->wValue);
        break;

    case SET_IDLE:
        hid->idlestate = (uint8_t)(req->wValue >> 8U);
        break;

    case SET_PROTOCOL:
        hid->protocol = (uint8_t)(req->wValue);
        break;

    case USB_GET_DESCRIPTOR:
        if (USB_DESCTYPE_REPORT == (req->wValue >> 8U))
        {
            switch (GetBridgeMode())
            {
                case MODE_PARALLEL_DIGITIZER:
                {
                    transc->remain_len = USB_MIN(DIGITIZER_REPORT_DESC_LEN, req->wLength);
                    transc->xfer_buf = (uint8_t *)digitizer_report_descriptor;
                    break;
                }

                case MODE_ABSOLUTE_MOUSE:
                {
                    transc->remain_len = USB_MIN(ABS_MOUSE_REPORT_DESC_LEN, req->wLength);
                    transc->xfer_buf = (uint8_t *)mouse_abs_report_descriptor;
                    break;
                }

                default:
                case MODE_TBP_BASIC:
                {
                    transc->remain_len = 0;
                    transc->xfer_buf = (uint8_t *)NULL;
                    break;
                }
            }
        }
        break;

    default:
        return USBD_FAIL;
    }

    return USBD_OK;
}

/*!
    \brief      handle digitizer HID data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t digitizer_hid_data_in(usb_dev *udev, uint8_t ep_num)
{
    digitizer_hid_handler *hhid = (digitizer_hid_handler *)udev->dev.class_data[DIGITIZER_INTERFACE_NUM];
    hhid->state = eIDLE;

    return USBD_OK;
}

/*!
    \brief      handle digitizer HID data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t digitizer_hid_data_out(usb_dev *udev, uint8_t ep_num)
{
    UNUSED(udev);
    UNUSED(ep_num);

    return USBD_OK;
}
