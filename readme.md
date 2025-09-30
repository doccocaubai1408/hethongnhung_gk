
void gui_lenh_lcd(char lenh)
{
	// xxxx bl en rw rs 
char bit_cao,bit_thap;
	uint8_t bit_gui[4];
	bit_cao = (lenh & 0xf0);
	bit_thap = ( (lenh<<4) & 0xf0); // chân D0-3 không dùng được ở chế độ 4 bit nên phải dịch bit từ bit thấp(nằm ở chân D0-D3-> cao(nằm ở chân D4-7)
	bit_gui[0] = bit_cao  | 0x0C; 
	bit_gui[1] = bit_cao  | 0x08;
	bit_gui[2] = bit_thap | 0x0C;
	bit_gui[3] = bit_thap | 0x08;
	HAL_I2C_Master_Transmit (&hi2c1, diachia_lcd,(uint8_t *) bit_gui, 4, 100);
}
void gui_data_lcd(char data )
{
	// xxxx bl en rw rs 
char bit_cao,bit_thap;
	uint8_t bit_gui[4];
	bit_cao = (data  & 0xf0);
	bit_thap = ( (data <<4) & 0xf0);
	bit_gui[0] = bit_cao  | 0x0D; //xxxx 1101 
	bit_gui[1] = bit_cao  | 0x09; // xxxx 1001
	bit_gui[2] = bit_thap | 0x0D;
	bit_gui[3] = bit_thap | 0x09;
	HAL_I2C_Master_Transmit (&hi2c1, diachia_lcd,(uint8_t *) bit_gui, 4, 100);
}

/*
lệnh 0x14 
Bit trong byte I2C,Chân PCF8574,Chân LCD tương ứng,Mô tả/Chức năng
Bit 0,P0,RS (Register Select),"Chọn chế độ: 0 = lệnh (command), 1 = dữ liệu (data)"
Bit 1,P1,RW (Read/Write),"Thường nối GND (0 = write), không dùng cho write mode"
Bit 2,P2,EN (Enable),"Tín hiệu kích hoạt: 1 để gửi lệnh/dữ liệu, sau đó về 0"
Bit 3,P3,BL (Backlight),"Đèn nền: 0 = tắt, 1 = bật"
Bit 4,P4,D4 (Data bit 4),Bit dữ liệu 4 của LCD (mode 4-bit)
Bit 5,P5,D5 (Data bit 5),Bit dữ liệu 5 của LCD
Bit 6,P6,D6 (Data bit 6),Bit dữ liệu 6 của LCD
Bit 7,P7,D7 (Data bit 7),Bit dữ liệu 7 của LCD
data_u = 00010100 & 11110000 =00010000
data_l = 01000000 & 11110000 = 01010000

data_t [0] = 0001 0000 or 0000 1100 = 0001 1100 = 0x1C
data_t [1] = 0001 0000 or 0000 1000 = 0001 1000 =0x18
data_t [2]= 0100 0000 or 0000 1100= 0100 1100 = 0x4C
data_t = 0100 0000 or 0000 1000 = 0100 1000 = 0x48
*/
void xoa_lcd  (void)
{
	gui_lenh_lcd (0x80);
	for (int i=0; i<70; i++)
	{
		gui_data_lcd (' ');
	}
}
/* 
đặt địa chỉ về vị trí đầu tiên của DDRAM 
ghi kí tự ' ' vào toàn bộ địa chỉ của DDRAM 
đối với 16x2 :
Dòng 1: 0x00 đến 0x0F 
Dòng 2: 0x40 đến 0x4F 
https://www.vishay.com/docs/37481/lcd016n002mttiet.pdf
tuy địa chỉ là 0x0- (đối với hàng 1 ) và 0x4- (đối với hàng 2) 
nhưng vì lệnh set địa chỉ của DDRAM của chip điều khiển lcd yêu cầu bit đầu tiên luôn =1 --> 1--- ----
=> set địa chỉ dòng 1: 0x80 đến 0x8F
=> set địa chỉ dòng 2 : 0xC0 đến 0xCF
 mỗi lần ghi kí tự địa chỉ thanh ghi sẽ tự dịch đến địa chỉ kế tiếp 
 
 */
void vi_tri(int hang,int cot)
{
	switch (hang)
	{
		case 0:
			cot |= 0x80;
break;		
		case 1:
			cot |= 0xC0;
break;
		}
	gui_lenh_lcd(cot);
}
/* row =0 ;col = 4 hàng 0 cột 4
=> case =0;
=> col = 0000 0100  or 1000 0000= 1000 0100 = 0x84 
 https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf
 */
void thiet_lap_lcd()
{
HAL_Delay(50);
// 8bit mode
gui_lenh_lcd(0x30);
HAL_Delay(5);
gui_lenh_lcd(0x30);
HAL_Delay(1);	
gui_lenh_lcd(0x30);
HAL_Delay(10);
// 4bit mode
gui_lenh_lcd(0x20);
HAL_Delay(10);
gui_lenh_lcd(0x28);
HAL_Delay(1);
gui_lenh_lcd(0x08);
	HAL_Delay(1);
gui_lenh_lcd(0x01);
	HAL_Delay(1);
	HAL_Delay(1);
	gui_lenh_lcd(0x06);
	HAL_Delay(1);
	gui_lenh_lcd(0x0C);
}
/* 
 https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf từ trang 25-28
 khởi tạo lcd mặc định 8 bit 
 đổi về chế độ 4 bit 
 đổi phông 5.8
 tắt màn hình 
 xóa màn hình 
 chế độ con trỏ lùi về phải ,màn hình không chạy 
 bật màn hình
 */
 void lcd_in_chuoi  (char *str)
{
	while (*str) gui_data_lcd (*str++);
}