#include "ST7735.h"
#include "fonts.h"

#ifdef USE_ST7735_MONITOR

uint8_t Gram[TotalPixel * 2] = {0};

void Compile_Test(void)
{
    int i = 1;
    i += i;
    if (i)
    {
        i++;
    }
}

uint16_t LCD_SetColor(uint32_t Color)
{
    uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;

    Red_Value = (uint16_t)((Color & 0x00F80000) >> 8);
    Green_Value = (uint16_t)((Color & 0x0000FC00) >> 5);
    Blue_Value = (uint16_t)((Color & 0x000000F8) >> 3);

    return (uint16_t)(Red_Value | Green_Value | Blue_Value);
}

void LCD_WriteCommand(uint8_t cmd)
{
    SetPortLcdDCCmd();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 1000);
    // HAL_SPI_Transmit_DMA(&hspi2, &cmd, 1);
}

void LCD_WriteByte(uint8_t data)
{
    SetPortLcdDCData();
    HAL_SPI_Transmit(&hspi2, &data, 1, 1000);
    // HAL_SPI_Transmit_DMA(&hspi2, &data, 1);
}

void LCD_WriteHalfWord(uint16_t data)
{
    uint8_t data_byte[2];
    data_byte[0] = data >> 8;
    data_byte[1] = data;
    SetPortLcdDCData();
    HAL_SPI_Transmit(&hspi2, data_byte, 2, 1000);
    // HAL_SPI_Transmit_DMA(&hspi2, data_byte, 2);
}

void LCD_SetRegion(uint16_t x, uint16_t x2, uint16_t y, uint16_t y2)
{
    LCD_WriteCommand(0x2A);
    LCD_WriteHalfWord(x);
    LCD_WriteHalfWord(x2);
    LCD_WriteCommand(0x2B);
    LCD_WriteHalfWord(y);
    LCD_WriteHalfWord(y2);
    LCD_WriteCommand(0x2C);
}

void LCD_Fill(uint16_t color)
{
    uint32_t i;
    LCD_SetRegion(0, 127, 0, 159);
    SetPortLcdDCData();
    for (i = 0; i < SizeOfGram;)
    {
        Gram[i] = color >> 8;
        Gram[i + 1] = color;
        i += 2;
    }
    // HAL_SPI_Transmit(&hspi2, (uint8_t *)Gram, TotalPixel * 2, 1000);
}

void LCD_Upgrade_Gram(void)
{
    LCD_SetRegion(0, LCD_W - 1, 0, LCD_H - 1);
    SetPortLcdDCData();
    // HAL_SPI_Transmit(&hspi2, (uint8_t *)Gram, SizeOfGram, 1000);
    HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)Gram, SizeOfGram);
}

void LCD_Fill_Region(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color)
{
    uint32_t i;
    uint32_t tempx = 2 * x1;
    uint32_t tempy = 0;
    uint32_t base_bias = 2 * LCD_W * y1 + 2 * x1;
    uint32_t iTotalPixel = (x2 - x1) * (y2 - y1);
    for (i = 0; i < iTotalPixel;)
    {
        Gram[base_bias + tempx] = color >> 8;
        Gram[base_bias + tempx + 1] = color;
        tempx += 2;
        if (tempx >= 2 * x2)
        {
            tempx = 2 * x1;
            tempy++;
            base_bias += (LCD_W * 2);
        }
        i++;
    }
    // LCD_Upgrade_Gram();
}

// void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
// {
//     LCD_SetRegion(x, x + 1, y, y + 1);
//     LCD_WriteHalfWord(color);
// }

void LCD_DrawPoint(uint32_t x, uint32_t y, uint16_t color)
{
    if (x > LCD_W - 1 || y > LCD_H - 1)
    {
        return;
    }
    Gram[x * 2 + y * LCD_W * 2] = color >> 8;
    Gram[x * 2 + y * LCD_W * 2 + 1] = color;
}

// glib库中的画线函数，可以画斜线，线两端分别是(x1, y1)和(x2, y2)
void LCD_DrawLine(uint32_t x1, uint32_t x2, uint32_t y1, uint32_t y2, uint16_t color)
{
    int dx, dy, e;
    dx = x2 - x1;
    dy = y2 - y1;

    if (dx >= 0)
    {
        if (dy >= 0) // dy>=0
        {
            if (dx >= dy) // 1/8 octant
            {
                e = dy - dx / 2;
                while (x1 <= x2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        y1 += 1;
                        e -= dx;
                    }
                    x1 += 1;
                    e += dy;
                }
            }
            else // 2/8 octant
            {
                e = dx - dy / 2;
                while (y1 <= y2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        x1 += 1;
                        e -= dy;
                    }
                    y1 += 1;
                    e += dx;
                }
            }
        }
        else // dy<0
        {
            dy = -dy;     // dy=abs(dy)
            if (dx >= dy) // 8/8 octant
            {
                e = dy - dx / 2;
                while (x1 <= x2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        y1 -= 1;
                        e -= dx;
                    }
                    x1 += 1;
                    e += dy;
                }
            }
            else // 7/8 octant
            {
                e = dx - dy / 2;
                while (y1 >= y2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        x1 += 1;
                        e -= dy;
                    }
                    y1 -= 1;
                    e += dx;
                }
            }
        }
    }
    else // dx<0
    {
        dx = -dx;    // dx=abs(dx)
        if (dy >= 0) // dy>=0
        {
            if (dx >= dy) // 4/8 octant
            {
                e = dy - dx / 2;
                while (x1 >= x2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        y1 += 1;
                        e -= dx;
                    }
                    x1 -= 1;
                    e += dy;
                }
            }
            else // 3/8 octant
            {
                e = dx - dy / 2;
                while (y1 <= y2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        x1 -= 1;
                        e -= dy;
                    }
                    y1 += 1;
                    e += dx;
                }
            }
        }
        else // dy<0
        {
            dy = -dy;     // dy=abs(dy)
            if (dx >= dy) // 5/8 octant
            {
                e = dy - dx / 2;
                while (x1 >= x2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        y1 -= 1;
                        e -= dx;
                    }
                    x1 -= 1;
                    e += dy;
                }
            }
            else // 6/8 octant
            {
                e = dx - dy / 2;
                while (y1 >= y2)
                {
                    LCD_DrawPoint(x1, y1, color);
                    if (e > 0)
                    {
                        x1 -= 1;
                        e -= dy;
                    }
                    y1 -= 1;
                    e += dx;
                }
            }
        }
    }
}

void LCD_DrawFromIMG(uint32_t x, uint32_t y, const uint8_t *IMG, uint8_t IMG_W, uint8_t IMG_H, uint16_t color)
{
    uint32_t TotalCounter = 0;
    uint32_t BitCounter = 0;
    uint32_t Buffer = 0;
    uint32_t tempx = x;
    uint32_t tempy = y;

    IMG_W += IMG_W % 8;

    IMG_H += IMG_H % 8;

    for (TotalCounter = 0; TotalCounter < IMG_W * IMG_H / 8;)
    {
        Buffer = *(IMG + TotalCounter);
        for (BitCounter = 0; BitCounter < 8;)
        {
            if (Buffer & 0x01)
            {
                LCD_DrawPoint(tempx, tempy, color);
            }

            tempx++;
            if (tempx >= x + IMG_W)
            {
                tempx = x;
                tempy++;
            }
            Buffer >>= 1;
            BitCounter++;
        }

        TotalCounter++;
    }
}

void LCD_DrawFromIMGWithBackGround(uint32_t x, uint32_t y, const uint8_t *IMG, uint8_t IMG_W, uint8_t IMG_H, uint16_t color, uint16_t BGcolor)
{

    uint32_t TotalCounter = 0;
    uint32_t BitCounter = 0;
    uint32_t Buffer = 0;
    uint32_t tempx = x;
    uint32_t tempy = y;

    IMG_W += IMG_W % 8;

    IMG_H += IMG_H % 8;

    for (TotalCounter = 0; TotalCounter < IMG_W * IMG_H / 8;)
    {
        Buffer = *(IMG + TotalCounter);
        for (BitCounter = 0; BitCounter < 8;)
        {
            if (Buffer & 0x01)
            {
                LCD_DrawPoint(tempx, tempy, color);
            }
            else
            {
                LCD_DrawPoint(tempx, tempy, BGcolor);
            }

            tempx++;
            if (tempx >= x + IMG_W)
            {
                tempx = x;
                tempy++;
            }
            Buffer >>= 1;
            BitCounter++;
        }

        TotalCounter++;
    }
}

void LCD_Print(uint32_t x, uint32_t y, uint32_t font_num, uint16_t color, char *fmt, ...)
{
    uint32_t temp = 0;

    uint32_t tempx = x;
    uint32_t tempy = y;
    uint32_t font_width;
    uint32_t font_height;
    uint32_t font_unit_size;
    uint8_t buf[100];

    unsigned char *pfont = NULL;
    uint16_t len = strlen((const char *)fmt);
    if (len > 100)
    {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)buf, fmt, ap);
    va_end(ap);
    switch (font_num)
    {
    default:
    case 1:
        /* code */
        pfont = (unsigned char *)ASCII_8_16;
        font_width = 8;
        font_height = 16;
        font_unit_size = 16;
        break;
    case 2:
        /* code */
        pfont = (unsigned char *)ASCII_12_32;
        font_width = 12;
        font_height = 32;
        font_unit_size = 96;
        break;
    case 3:
        pfont = (unsigned char *)JetBrainMono;
        font_width = 8;
        font_height = 16;
        font_unit_size = 16;
        break;
    }
    for (temp = 0; temp < len;)
    {
        if (tempx >= LCD_W - font_width)
        {
            tempx = x;
            tempy += font_height;
        }
        if (buf[temp] == '\r' | buf[temp] == '\n')
        {
            temp++;
            tempx = x;
            tempy += font_height;
        }
        if (buf[temp] == '\0')
        {
            break;
        }
        LCD_DrawFromIMG(tempx, tempy, pfont + ((buf[temp] - 32) * (font_unit_size)), font_width, font_height, color);
        tempx += font_width;

        temp++;
    }
}

void LCD_PrintWithBackGroundColor(uint32_t x, uint32_t y, uint32_t font_num, uint16_t color, uint16_t BGcolor, char *fmt, ...)
{
    uint32_t temp = 0;

    uint32_t tempx = x;
    uint32_t tempy = y;
    uint32_t font_width;
    uint32_t font_height;
    uint32_t font_unit_size;

    //    uint32_t xnum;
    //    uint32_t ynum;
    uint8_t buf[200];

    unsigned char *pfont = NULL;
    uint16_t len = strlen((const char *)fmt);
    if (len > 200)
    {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)buf, fmt, ap);
    va_end(ap);
    switch (font_num)
    {
    default:
    case 1:
        /* code */
        pfont = (unsigned char *)ASCII_8_16;
        font_width = 8;
        font_height = 16;
        font_unit_size = 16;
        break;
    case 2:
        /* code */
        pfont = (unsigned char *)ASCII_12_32;
        font_width = 12;
        font_height = 32;
        font_unit_size = 96;
        break;
    case 3:
        pfont = (unsigned char *)JetBrainMono;
        font_width = 8;
        font_height = 16;
        font_unit_size = 16;
        break;
    }

    //    xnum = (LCD_W - x) / font_width;
    //    ynum = 1 + len / xnum;
    for (temp = 0; temp < len;)
    {
        if (tempx >= LCD_W - font_width)
        {
            tempx = x;
            tempy += font_height;
        }
        if (buf[temp] == '\r' | buf[temp] == '\n')
        {
            temp++;
            tempx = x;
            tempy += font_height;
        }
        if (buf[temp] == '\0')
        {
            break;
        }
        LCD_DrawFromIMGWithBackGround(tempx, tempy, pfont + ((buf[temp] - 32) * (font_unit_size)), font_width, font_height, color, BGcolor);
        tempx += font_width;

        temp++;
    }
}

void LCD_DrawPic(const unsigned char *pic, uint32_t W, uint32_t H)
{
    uint32_t i = 0;
    for (i = 0; i < W * H * 2;)
    {
        Gram[i] = pic[i];
        i++;
    }
}

static void LCD_CirclePlot(uint32_t x, uint32_t y, uint32_t xi, uint32_t yi, uint16_t color)
{
    LCD_DrawPoint(x + xi, y + yi, color);
    LCD_DrawPoint(x + yi, y + xi, color);
    LCD_DrawPoint(x - xi, y + yi, color);
    LCD_DrawPoint(x - yi, y + xi, color);
    LCD_DrawPoint(x - xi, y - yi, color);
    LCD_DrawPoint(x - yi, y - xi, color);
    LCD_DrawPoint(x + xi, y - yi, color);
    LCD_DrawPoint(x + yi, y - xi, color);
}

void LCD_FastCircle(uint32_t x, uint32_t y, uint32_t r, uint32_t color)
{
    uint16_t Tcolor = TransColor888to565(color);

    int xi;
    int yi;
    int di;
    if ((int32_t)(x - r) < 0 || x + r > LCD_W || (int32_t)(y - r) < 0 || y + r > LCD_H)
    {
        return;
    }
    di = 0 - (r >> 1);
    xi = 0;
    yi = r;
    while (yi >= xi)
    {
        LCD_CirclePlot(x, y, xi, yi, Tcolor);
        xi++;
        if (di < 0)
        {
            di += xi;
        }
        else
        {
            yi--;
            di += xi - yi;
        }
    }
}

void LCD_FillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint32_t color)
{
    uint16_t Tcolor = TransColor888to565(color);
    int x, y;
    int deltax, deltay;
    int d;
    int xi;
    x = 0;
    y = r;
    deltax = 3;
    deltay = 2 - r - r;
    d = 1 - r;

    LCD_DrawPoint(x + x0, y + y0, Tcolor);
    LCD_DrawPoint(x + x0, -y + y0, Tcolor);
    for (xi = -r + x0; xi <= r + x0; xi++)
        LCD_DrawPoint(xi, y0, Tcolor); // 水平线填充
    while (x < y)
    {
        if (d < 0)
        {
            d += deltax;
            deltax += 2;
            x++;
        }
        else
        {
            d += (deltax + deltay);
            deltax += 2;
            deltay += 2;
            x++;
            y--;
        }
        for (xi = -x + x0; xi <= x + x0; xi++)
        {
            LCD_DrawPoint(xi, -y + y0, Tcolor);
            LCD_DrawPoint(xi, y + y0, Tcolor); // 扫描线填充
        }
        for (xi = -y + x0; xi <= y + x0; xi++)
        {
            LCD_DrawPoint(xi, -x + y0, Tcolor);
            LCD_DrawPoint(xi, x + y0, Tcolor); // 扫描线填充其量
        }
    }
}

static int32_t ocs_data[LCD_W];
static uint32_t schedule;
static int32_t ocs_max, ocs_min;
static fp32 ocs_propotion;
void LCD_OCS(int32_t data)
{

    ocs_data[schedule] = data;
    for (uint32_t i = 0; i < LCD_W;)
    {
        // LCD_DrawPoint(i, ocs_data[i] + 80, 0x0000);
        if (data >= 80)
        {
            ocs_max = data;
            ocs_propotion = 160 / (ocs_max - ocs_min);
        }
        else if (data <= -80)
        {
            ocs_min = data;
            ocs_propotion = 160 / (ocs_max - ocs_min);
        }
        if (i == 0)
        {
            LCD_DrawPoint(i, ocs_propotion * ocs_data[i] + 80, 0x0000);
            i++;
            continue;
        }
        if (i < schedule)
        {
            LCD_DrawLine(i - 1, i, ocs_data[i - 1] + 80, ocs_propotion * ocs_data[i] + 80, 0x0000);
        }
        i++;
    }

    schedule++;
    if (schedule >= LCD_W)
    {
        schedule = 0;
        // memset(ocs_data, 0, LCD_W);
    }
}

void Lcd_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    //----ST7735S reset sequence----------------//
    SetPortLcdResetHigh();
    lcd_delay_ms(5);
    SetPortLcdResetLow();
    lcd_delay_ms(5);
    SetPortLcdResetHigh();
    lcd_delay_ms(20);

    LCD_WriteCommand(0x01); // SWRESET Software reset
    lcd_delay_ms(20);

    LCD_WriteCommand(0x11); // Sleep out
    lcd_delay_ms(50);

    //----ST7735S Frame Rate---------------------//
    LCD_WriteCommand(0xB1); // Frame rate 80Hz Frame rate=333k/((RTNA + 20) x (LINE + FPA + BPA))
    LCD_WriteByte(0x05);    // RTNA
    LCD_WriteByte(0x3A);    // FPA
    LCD_WriteByte(0x3A);    // BPA
    LCD_WriteCommand(0xB2); // Frame rate 80Hz
    LCD_WriteByte(0x05);
    LCD_WriteByte(0x3A);
    LCD_WriteByte(0x3A);
    LCD_WriteCommand(0xB3); // Frame rate 80Hz
    LCD_WriteByte(0x05);
    LCD_WriteByte(0x3A);
    LCD_WriteByte(0x3A);
    LCD_WriteByte(0x05);
    LCD_WriteByte(0x3A);
    LCD_WriteByte(0x3A);

    //------------------------------------Display Inversion Control-----------------------------------------//
    LCD_WriteCommand(0xB4);
    LCD_WriteByte(0x03);

    //------------------------------------ST7735S Power Sequence-----------------------------------------//
    LCD_WriteCommand(0xC0);
    LCD_WriteByte(0x62);
    LCD_WriteByte(0x02);
    LCD_WriteByte(0x04);
    LCD_WriteCommand(0xC1);
    LCD_WriteByte(0xC0);
    LCD_WriteCommand(0xC2);
    LCD_WriteByte(0x0D);
    LCD_WriteByte(0x00);
    LCD_WriteCommand(0xC3);
    LCD_WriteByte(0x8D);
    LCD_WriteByte(0x6A);
    LCD_WriteCommand(0xC4);
    LCD_WriteByte(0x8D);
    LCD_WriteByte(0xEE);
    //---------------------------------End ST7735S Power Sequence---------------------------------------//
    LCD_WriteCommand(0xC5); // VCOM
    LCD_WriteByte(0x0E);
    LCD_WriteCommand(0x36); // MX, MY, RGB mode
    LCD_WriteByte(0x00);
    //------------------------------------ST7735S Gamma Sequence-----------------------------------------//
    LCD_WriteCommand(0XE0);
    LCD_WriteByte(0x10);
    LCD_WriteByte(0x0E);
    LCD_WriteByte(0x02);
    LCD_WriteByte(0x03);
    LCD_WriteByte(0x0E);
    LCD_WriteByte(0x07);
    LCD_WriteByte(0x02);
    LCD_WriteByte(0x07);
    LCD_WriteByte(0x0A);
    LCD_WriteByte(0x12);
    LCD_WriteByte(0x27);
    LCD_WriteByte(0x37);
    LCD_WriteByte(0x00);
    LCD_WriteByte(0x0D);
    LCD_WriteByte(0x0E);
    LCD_WriteByte(0x10);
    LCD_WriteCommand(0XE1);
    LCD_WriteByte(0x10);
    LCD_WriteByte(0x0E);
    LCD_WriteByte(0x03);
    LCD_WriteByte(0x03);
    LCD_WriteByte(0x0F);
    LCD_WriteByte(0x06);
    LCD_WriteByte(0x02);
    LCD_WriteByte(0x08);
    LCD_WriteByte(0x0A);
    LCD_WriteByte(0x13);
    LCD_WriteByte(0x26);
    LCD_WriteByte(0x36);
    LCD_WriteByte(0x00);
    LCD_WriteByte(0x0D);
    LCD_WriteByte(0x0E);
    LCD_WriteByte(0x10);
    //------------------------------------End ST773F5S Gamma Sequence-----------------------------------------//
    LCD_WriteCommand(0x3A); // 65k mode
    LCD_WriteByte(0x05);
    LCD_WriteCommand(0x29); // Display on
    lcd_delay_ms(50);
}

void Monitor_Frame_Upgrade(void *arg)
{
    BaseType_t Frame_Period = 16;
    motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(0);

    motor_measure_t *motinfo4 = (motor_measure_t *)get_measure_pointer(4);
    motor_measure_t *motinfo5 = (motor_measure_t *)get_measure_pointer(5);
    const RC_ctrl_t *pxDR16_noredef = get_remote_control_point();

    PID *pxYAW_P = pid_get_struct_pointer(1, YAW_MOTOR);
    //    fp32 *pfacc = get_imu_data_pointer(RETURN_YAW);
    TickType_t tick = xTaskGetTickCount();

    chassis_transported_controller_data *rc_chas = get_rc_data_from_chassis_pointer();
    LCD_Fill(0x0000);
    for (;;)
    {
        LCD_Fill(TransColor888to565(0x002927FF));
        LCD_PrintWithBackGroundColor(0, 0, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "Pit:%0.3f  ", INS.Pitch);
        LCD_PrintWithBackGroundColor(0, 16, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "Yaw:%0.3f  ", INS.Yaw);
        // LCD_PrintWithBackGroundColor(0, 32, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "Rol:%0.3f   ", INS.Roll);
        // LCD_PrintWithBackGroundColor(0, 48, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "ambt:%+0.3f   ", pc_aimbot_data.yaw);
        //        LCD_PrintWithBackGroundColor(0, 48, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "TIK?:%d        ", tick / 1000);
        //        tick = xTaskGetTickCount();

        LCD_PrintWithBackGroundColor(0, 64, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "key:%d  ", rc_chas->key.v);
        LCD_PrintWithBackGroundColor(0, 80, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "2:%f      ", pc_aimbot_data.yaw);
        LCD_PrintWithBackGroundColor(0, 96, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "3:%f      ", pc_aimbot_data.pit);

        // LCD_PrintWithBackGroundColor(0, 96, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "VLD:%d  ", rc_chas->mouse.x);

        //        // LCD_PrintWithBackGroundColor(0, 64, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "g_yaw:%0.3f  ", 0.123f);

        //        switch (pxDR16_noredef->controller.S2)
        //        {
        //        case 1:
        //            LCD_PrintWithBackGroundColor(0, 32, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "Mode:Halt");
        //            break;
        //        case 3:
        //            LCD_PrintWithBackGroundColor(0, 32, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "Mode:Slow");
        //            break;
        //        case 2:
        //            LCD_PrintWithBackGroundColor(0, 32, 3, TransColor888to565(0x00FDFF27), TransColor888to565(0x002927FF), "Mode:Fast");
        //            break;
        //        default:
        //            break;
        //        }

        //        switch (pxDR16_noredef->controller.S1)
        //        {
        //        case 1:
        //            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
        //            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
        //            break;
        //        case 3:
        //            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
        //            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
        //            break;
        //        case 2:
        //            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
        //            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
        //            break;
        //        default:
        //            break;
        //        }
        // LCD_OCS(pxYAW_P->actual);
        LCD_FastCircle(20, 120, 20, 0x00206CC00);
        LCD_FillCircle(20 - (1024 - pxDR16_noredef->rc.ch[2]) / 33, 120 + (1024 - pxDR16_noredef->rc.ch[3]) / 33, 5, 0x00FF0000);
        LCD_FastCircle(108, 120, 20, 0x00206CC00);
        LCD_FillCircle(108 - (1024 - pxDR16_noredef->rc.ch[0]) / 33, 120 + (1024 - pxDR16_noredef->rc.ch[1]) / 33, 5, 0x00FF0000);
        LCD_Upgrade_Gram();
        // USART_Print("debug");
        vTaskDelay(Frame_Period);
    }
};

#endif // USE_ST7735_MONITOR