#include <INFANTRY_picture.h>


void static_picture(UART_HandleTypeDef  *huart,uint16_t robot_id)
{
	static ext_client_custom_graphic_seven_t draw_picture;
	static uint8_t Show_Pack[150] = {0};
	picture_data_head Picture_Header;
	ext_data_frame_header_t header;
	
	header.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header.DATA_LENGTH=D7_LEN;
	header.SEQ=0;
	memcpy(Show_Pack,(uint8_t *)&header,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack,HEADER_LEN);
	
	Picture_Header.Cmd_Id=0x0301;
	Picture_Header.Student_Interactive_Header_Data.data_cmd_id=D7_ID;
	Picture_Header.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack+HEADER_LEN,(uint8_t *)&Picture_Header,sizeof(Picture_Header));
	
	draw_picture.grapic_data_struct[0].graphic_name[0]=0;
	draw_picture.grapic_data_struct[0].graphic_name[1]=0;
	draw_picture.grapic_data_struct[0].graphic_name[2]=1;
	draw_picture.grapic_data_struct[0].operate_tpye=1;
	draw_picture.grapic_data_struct[0].graphic_tpye=2;
	draw_picture.grapic_data_struct[0].layer=0;
	draw_picture.grapic_data_struct[0].color=1;
	draw_picture.grapic_data_struct[0].start_angle=0;
	draw_picture.grapic_data_struct[0].end_angle=0;
	draw_picture.grapic_data_struct[0].width=4;
	draw_picture.grapic_data_struct[0].start_x=960;
	draw_picture.grapic_data_struct[0].start_y=F0_Y;
	draw_picture.grapic_data_struct[0].radius=60;
	draw_picture.grapic_data_struct[0].end_x=0;
	draw_picture.grapic_data_struct[0].end_y=0;//圆
	
	draw_picture.grapic_data_struct[1].graphic_name[0]=0;
	draw_picture.grapic_data_struct[1].graphic_name[1]=0;
	draw_picture.grapic_data_struct[1].graphic_name[2]=2;
	draw_picture.grapic_data_struct[1].operate_tpye=1;
	draw_picture.grapic_data_struct[1].graphic_tpye=0;
	draw_picture.grapic_data_struct[1].layer=0;
	draw_picture.grapic_data_struct[1].color=1;
	draw_picture.grapic_data_struct[1].start_angle=0;
	draw_picture.grapic_data_struct[1].end_angle=0;
	draw_picture.grapic_data_struct[1].width=1;
	draw_picture.grapic_data_struct[1].start_x=960;
	draw_picture.grapic_data_struct[1].start_y=F0_Y;
	draw_picture.grapic_data_struct[1].radius=0;
	draw_picture.grapic_data_struct[1].end_x=960;
	draw_picture.grapic_data_struct[1].end_y=F0_Y-300;//竖线
	
	draw_picture.grapic_data_struct[2].graphic_name[0]=0;
	draw_picture.grapic_data_struct[2].graphic_name[1]=0;
	draw_picture.grapic_data_struct[2].graphic_name[2]=3;
	draw_picture.grapic_data_struct[2].operate_tpye=1;
	draw_picture.grapic_data_struct[2].graphic_tpye=0;
	draw_picture.grapic_data_struct[2].layer=0;
	draw_picture.grapic_data_struct[2].color=1;
	draw_picture.grapic_data_struct[2].start_angle=0;
	draw_picture.grapic_data_struct[2].end_angle=0;
	draw_picture.grapic_data_struct[2].width=1;
	draw_picture.grapic_data_struct[2].start_x=960-0.5*F0_LEN;
	draw_picture.grapic_data_struct[2].start_y=F0_Y;
	draw_picture.grapic_data_struct[2].radius=0;
	draw_picture.grapic_data_struct[2].end_x=960+0.5*F0_LEN;
	draw_picture.grapic_data_struct[2].end_y=F0_Y;//0
	
	draw_picture.grapic_data_struct[3].graphic_name[0]=0;
	draw_picture.grapic_data_struct[3].graphic_name[1]=0;
	draw_picture.grapic_data_struct[3].graphic_name[2]=4;
	draw_picture.grapic_data_struct[3].operate_tpye=1;
	draw_picture.grapic_data_struct[3].graphic_tpye=0;
	draw_picture.grapic_data_struct[3].layer=0;
	draw_picture.grapic_data_struct[3].color=1;
	draw_picture.grapic_data_struct[3].start_angle=0;
	draw_picture.grapic_data_struct[3].end_angle=0;
	draw_picture.grapic_data_struct[3].width=1;
	draw_picture.grapic_data_struct[3].start_x=960-0.5*F100_LEN;
	draw_picture.grapic_data_struct[3].start_y=F100_Y;
	draw_picture.grapic_data_struct[3].radius=0;
	draw_picture.grapic_data_struct[3].end_x=960+0.5*F100_LEN;
	draw_picture.grapic_data_struct[3].end_y=F100_Y;//100
	
	draw_picture.grapic_data_struct[4].graphic_name[0]=0;
	draw_picture.grapic_data_struct[4].graphic_name[1]=0;
	draw_picture.grapic_data_struct[4].graphic_name[2]=5;
	draw_picture.grapic_data_struct[4].operate_tpye=1;
	draw_picture.grapic_data_struct[4].graphic_tpye=0;
	draw_picture.grapic_data_struct[4].layer=0;
	draw_picture.grapic_data_struct[4].color=1;
	draw_picture.grapic_data_struct[4].start_angle=0;
	draw_picture.grapic_data_struct[4].end_angle=0;
	draw_picture.grapic_data_struct[4].width=1;
	draw_picture.grapic_data_struct[4].start_x=960-0.5*F200_LEN;
	draw_picture.grapic_data_struct[4].start_y=F200_Y;
	draw_picture.grapic_data_struct[4].radius=0;
	draw_picture.grapic_data_struct[4].end_x=960+0.5*F200_LEN;
	draw_picture.grapic_data_struct[4].end_y=F200_Y;//200
	
	draw_picture.grapic_data_struct[5].graphic_name[0]=0;
	draw_picture.grapic_data_struct[5].graphic_name[1]=0;
	draw_picture.grapic_data_struct[5].graphic_name[2]=6;
	draw_picture.grapic_data_struct[5].operate_tpye=1;
	draw_picture.grapic_data_struct[5].graphic_tpye=0;
	draw_picture.grapic_data_struct[5].layer=0;
	draw_picture.grapic_data_struct[5].color=1;
	draw_picture.grapic_data_struct[5].start_angle=0;
	draw_picture.grapic_data_struct[5].end_angle=0;
	draw_picture.grapic_data_struct[5].width=1;
	draw_picture.grapic_data_struct[5].start_x=960-0.5*F300_LEN;
	draw_picture.grapic_data_struct[5].start_y=F300_Y;
	draw_picture.grapic_data_struct[5].radius=0;
	draw_picture.grapic_data_struct[5].end_x=960+0.5*F300_LEN;
	draw_picture.grapic_data_struct[5].end_y=F300_Y;//300
	
	draw_picture.grapic_data_struct[6].graphic_name[0]=0;
	draw_picture.grapic_data_struct[6].graphic_name[1]=0;
	draw_picture.grapic_data_struct[6].graphic_name[2]=7;
	draw_picture.grapic_data_struct[6].operate_tpye=1;
	draw_picture.grapic_data_struct[6].graphic_tpye=0;
	draw_picture.grapic_data_struct[6].layer=0;
	draw_picture.grapic_data_struct[6].color=1;
	draw_picture.grapic_data_struct[6].start_angle=0;
	draw_picture.grapic_data_struct[6].end_angle=0;
	draw_picture.grapic_data_struct[6].width=1;
	draw_picture.grapic_data_struct[6].start_x=960-0.5*F400_LEN;
	draw_picture.grapic_data_struct[6].start_y=F400_Y;
	draw_picture.grapic_data_struct[6].radius=0;
	draw_picture.grapic_data_struct[6].end_x=960+0.5*F400_LEN;
	draw_picture.grapic_data_struct[6].end_y=F400_Y;//400
	
	memcpy(Show_Pack+HEADER_LEN+sizeof(Picture_Header),(uint8_t *)&draw_picture,sizeof(draw_picture));
	Append_CRC16_Check_Sum_p(Show_Pack,sizeof(draw_picture)+HEADER_LEN+sizeof(Picture_Header)+CRC_LEN);
	
	HAL_UART_Transmit(huart,Show_Pack,sizeof(draw_picture)+HEADER_LEN+sizeof(Picture_Header)+CRC_LEN,0xff);
	
	delay(100);
	
	static ext_client_custom_character_t draw_char;
	static uint8_t Show_Pack2[150] = {0};
	picture_data_head Picture_Header2;
	ext_data_frame_header_t header2;
	
	header2.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header2.DATA_LENGTH=DCHAR_LEN;
	header2.SEQ=0;
	memcpy(Show_Pack2,(uint8_t *)&header2,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack2,HEADER_LEN);
	
	Picture_Header2.Cmd_Id=0x0301;
	Picture_Header2.Student_Interactive_Header_Data.data_cmd_id=DCHAR_ID;
	Picture_Header2.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header2.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack2+HEADER_LEN,(uint8_t *)&Picture_Header2,sizeof(Picture_Header2));
	
	draw_char.grapic_data_struct.graphic_name[0]=0;
	draw_char.grapic_data_struct.graphic_name[1]=0;
	draw_char.grapic_data_struct.graphic_name[2]=8;
	draw_char.grapic_data_struct.operate_tpye=1;
	draw_char.grapic_data_struct.graphic_tpye=7;
	draw_char.grapic_data_struct.layer=1;
	draw_char.grapic_data_struct.color=1;
	draw_char.grapic_data_struct.start_angle=20;
	draw_char.grapic_data_struct.end_angle=8;
	draw_char.grapic_data_struct.width=2;
	draw_char.grapic_data_struct.start_x=sc_X;
	draw_char.grapic_data_struct.start_y=sc_Y;
	draw_char.grapic_data_struct.radius=0;
	draw_char.grapic_data_struct.end_x=0;
	draw_char.grapic_data_struct.end_y=0;
	char data[]={'S','u','p','e','r','C','a','p'};
	memcpy(draw_char.data,(uint8_t *)&data,8);
	memcpy(Show_Pack2+HEADER_LEN+sizeof(Picture_Header2),(uint8_t *)&draw_char,sizeof(draw_char));
	Append_CRC16_Check_Sum_p(Show_Pack2,sizeof(draw_char)+HEADER_LEN+sizeof(Picture_Header2)+CRC_LEN);
	
	
	HAL_UART_Transmit(huart,Show_Pack2,sizeof(draw_char)+HEADER_LEN+sizeof(Picture_Header2)+CRC_LEN,0xff);
	
	delay(100);
	
	static ext_client_custom_character_t draw_char2;
	static uint8_t Show_Pack3[100] = {0};
	picture_data_head Picture_Header3;
	ext_data_frame_header_t header3;
	
	header3.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header3.DATA_LENGTH=DCHAR_LEN;
	header3.SEQ=0;
	memcpy(Show_Pack3,(uint8_t *)&header3,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack3,HEADER_LEN);
	
	Picture_Header3.Cmd_Id=0x0301;
	Picture_Header3.Student_Interactive_Header_Data.data_cmd_id=DCHAR_ID;
	Picture_Header3.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header3.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack3+HEADER_LEN,(uint8_t *)&Picture_Header3,sizeof(Picture_Header3));
	
	draw_char2.grapic_data_struct.graphic_name[0]=0;
	draw_char2.grapic_data_struct.graphic_name[1]=0;
	draw_char2.grapic_data_struct.graphic_name[2]=9;
	draw_char2.grapic_data_struct.operate_tpye=1;
	draw_char2.grapic_data_struct.graphic_tpye=7;
	draw_char2.grapic_data_struct.layer=2;
	draw_char2.grapic_data_struct.color=1;
	draw_char2.grapic_data_struct.start_angle=20;
	draw_char2.grapic_data_struct.end_angle=10;
	draw_char2.grapic_data_struct.width=2;
	draw_char2.grapic_data_struct.start_x=st_X;
	draw_char2.grapic_data_struct.start_y=st_Y;
	draw_char2.grapic_data_struct.radius=0;
	draw_char2.grapic_data_struct.end_x=0;
	draw_char2.grapic_data_struct.end_y=0;
	char data2[]="Q E R V SH";
	memcpy(draw_char2.data,(uint8_t *)&data2,10);
	memcpy(Show_Pack3+HEADER_LEN+sizeof(Picture_Header3),(uint8_t *)&draw_char2,sizeof(draw_char2));
	Append_CRC16_Check_Sum_p(Show_Pack3,sizeof(draw_char2)+HEADER_LEN+sizeof(Picture_Header3)+CRC_LEN);
	
	
	HAL_UART_Transmit(huart,Show_Pack3,sizeof(draw_char2)+HEADER_LEN+sizeof(Picture_Header3)+CRC_LEN,0xff);
	
	delay(100);
	
	static ext_client_custom_graphic_five_t draw_picture2;
	static uint8_t Show_Pack4[100] = {0};
	picture_data_head Picture_Header4;
	ext_data_frame_header_t header4;
	
	header4.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header4.DATA_LENGTH=D5_LEN;
	header4.SEQ=0;
	memcpy(Show_Pack4,(uint8_t *)&header4,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack4,HEADER_LEN);
	
	Picture_Header4.Cmd_Id=0x0301;
	Picture_Header4.Student_Interactive_Header_Data.data_cmd_id=D5_ID;
	Picture_Header4.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header4.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack4+HEADER_LEN,(uint8_t *)&Picture_Header4,sizeof(Picture_Header4));
	
	draw_picture2.grapic_data_struct[0].graphic_name[0]=0;
	draw_picture2.grapic_data_struct[0].graphic_name[1]=1;
	draw_picture2.grapic_data_struct[0].graphic_name[2]=0;
	draw_picture2.grapic_data_struct[0].operate_tpye=1;
	draw_picture2.grapic_data_struct[0].graphic_tpye=1;
	draw_picture2.grapic_data_struct[0].layer=0;
	draw_picture2.grapic_data_struct[0].color=1;
	draw_picture2.grapic_data_struct[0].start_angle=0;
	draw_picture2.grapic_data_struct[0].end_angle=0;
	draw_picture2.grapic_data_struct[0].width=2;
	draw_picture2.grapic_data_struct[0].start_x=st_X;
	draw_picture2.grapic_data_struct[0].start_y=st_Y-25;
	draw_picture2.grapic_data_struct[0].radius=60;
	draw_picture2.grapic_data_struct[0].end_x=st_X+20;
	draw_picture2.grapic_data_struct[0].end_y=st_Y-45;//Q
	
	draw_picture2.grapic_data_struct[1].graphic_name[0]=0;
	draw_picture2.grapic_data_struct[1].graphic_name[1]=1;
	draw_picture2.grapic_data_struct[1].graphic_name[2]=1;
	draw_picture2.grapic_data_struct[1].operate_tpye=1;
	draw_picture2.grapic_data_struct[1].graphic_tpye=1;
	draw_picture2.grapic_data_struct[1].layer=0;
	draw_picture2.grapic_data_struct[1].color=1;
	draw_picture2.grapic_data_struct[1].start_angle=0;
	draw_picture2.grapic_data_struct[1].end_angle=0;
	draw_picture2.grapic_data_struct[1].width=2;
	draw_picture2.grapic_data_struct[1].start_x=st_X+40;
	draw_picture2.grapic_data_struct[1].start_y=st_Y-25;
	draw_picture2.grapic_data_struct[1].radius=0;
	draw_picture2.grapic_data_struct[1].end_x=st_X+60;
	draw_picture2.grapic_data_struct[1].end_y=st_Y-45;//E
	
	draw_picture2.grapic_data_struct[2].graphic_name[0]=0;
	draw_picture2.grapic_data_struct[2].graphic_name[1]=1;
	draw_picture2.grapic_data_struct[2].graphic_name[2]=2;
	draw_picture2.grapic_data_struct[2].operate_tpye=1;
	draw_picture2.grapic_data_struct[2].graphic_tpye=1;
	draw_picture2.grapic_data_struct[2].layer=0;
	draw_picture2.grapic_data_struct[2].color=1;
	draw_picture2.grapic_data_struct[2].start_angle=0;
	draw_picture2.grapic_data_struct[2].end_angle=0;
	draw_picture2.grapic_data_struct[2].width=2;
	draw_picture2.grapic_data_struct[2].start_x=st_X+80;
	draw_picture2.grapic_data_struct[2].start_y=st_Y-25;
	draw_picture2.grapic_data_struct[2].radius=0;
	draw_picture2.grapic_data_struct[2].end_x=st_X+100;
	draw_picture2.grapic_data_struct[2].end_y=st_Y-45;//R
	
	draw_picture2.grapic_data_struct[3].graphic_name[0]=0;
	draw_picture2.grapic_data_struct[3].graphic_name[1]=1;
	draw_picture2.grapic_data_struct[3].graphic_name[2]=3;
	draw_picture2.grapic_data_struct[3].operate_tpye=1;
	draw_picture2.grapic_data_struct[3].graphic_tpye=1;
	draw_picture2.grapic_data_struct[3].layer=0;
	draw_picture2.grapic_data_struct[3].color=1;
	draw_picture2.grapic_data_struct[3].start_angle=0;
	draw_picture2.grapic_data_struct[3].end_angle=0;
	draw_picture2.grapic_data_struct[3].width=2;
	draw_picture2.grapic_data_struct[3].start_x=st_X+120;
	draw_picture2.grapic_data_struct[3].start_y=st_Y-25;
	draw_picture2.grapic_data_struct[3].radius=0;
	draw_picture2.grapic_data_struct[3].end_x=st_X+140;
	draw_picture2.grapic_data_struct[3].end_y=st_Y-45;//V
	
	draw_picture2.grapic_data_struct[4].graphic_name[0]=0;
	draw_picture2.grapic_data_struct[4].graphic_name[1]=1;
	draw_picture2.grapic_data_struct[4].graphic_name[2]=4;
	draw_picture2.grapic_data_struct[4].operate_tpye=1;
	draw_picture2.grapic_data_struct[4].graphic_tpye=1;
	draw_picture2.grapic_data_struct[4].layer=0;
	draw_picture2.grapic_data_struct[4].color=1;
	draw_picture2.grapic_data_struct[4].start_angle=0;
	draw_picture2.grapic_data_struct[4].end_angle=0;
	draw_picture2.grapic_data_struct[4].width=2;
	draw_picture2.grapic_data_struct[4].start_x=st_X+160;
	draw_picture2.grapic_data_struct[4].start_y=st_Y-25;
	draw_picture2.grapic_data_struct[4].radius=0;
	draw_picture2.grapic_data_struct[4].end_x=st_X+180;
	draw_picture2.grapic_data_struct[4].end_y=st_Y-45;//SH
	
	
	memcpy(Show_Pack4+HEADER_LEN+sizeof(Picture_Header4),(uint8_t *)&draw_picture2,sizeof(draw_picture2));
	Append_CRC16_Check_Sum_p(Show_Pack4,sizeof(draw_picture2)+HEADER_LEN+sizeof(Picture_Header4)+CRC_LEN);
	
	HAL_UART_Transmit(huart,Show_Pack4,sizeof(draw_picture2)+HEADER_LEN+sizeof(Picture_Header4)+CRC_LEN,0xff);
}

void flash_picture_init(UART_HandleTypeDef  *huart,uint16_t robot_id)
{
	uint16_t supercap=20;
//	float distance=15;
	
	static ext_client_custom_graphic_seven_t draw_picture;
	static uint8_t Show_Pack[150] = {0};
	picture_data_head Picture_Header;
	ext_data_frame_header_t header;
	
	header.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header.DATA_LENGTH=D7_LEN;
	header.SEQ=0;
	memcpy(Show_Pack,(uint8_t *)&header,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack,HEADER_LEN);
	
	Picture_Header.Cmd_Id=0x0301;
	Picture_Header.Student_Interactive_Header_Data.data_cmd_id=D7_ID;
	Picture_Header.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack+HEADER_LEN,(uint8_t *)&Picture_Header,sizeof(Picture_Header));
	
	
	draw_picture.grapic_data_struct[0].graphic_name[0]=0;
	draw_picture.grapic_data_struct[0].graphic_name[1]=1;
	draw_picture.grapic_data_struct[0].graphic_name[2]=5;
	draw_picture.grapic_data_struct[0].operate_tpye=1;
	draw_picture.grapic_data_struct[0].graphic_tpye=4;
	draw_picture.grapic_data_struct[0].layer=3;
	draw_picture.grapic_data_struct[0].color=2;
	draw_picture.grapic_data_struct[0].start_angle=0;
	draw_picture.grapic_data_struct[0].end_angle=300;
	draw_picture.grapic_data_struct[0].width=5;
	draw_picture.grapic_data_struct[0].start_x=sc_X+200;
	draw_picture.grapic_data_struct[0].start_y=sc_Y-5;
	draw_picture.grapic_data_struct[0].radius=0;
	draw_picture.grapic_data_struct[0].end_x=30;
	draw_picture.grapic_data_struct[0].end_y=30;//电容显示
	
	draw_picture.grapic_data_struct[1].graphic_name[0]=0;
	draw_picture.grapic_data_struct[1].graphic_name[1]=1;
	draw_picture.grapic_data_struct[1].graphic_name[2]=6;
	draw_picture.grapic_data_struct[1].operate_tpye=1;
	draw_picture.grapic_data_struct[1].graphic_tpye=1;
	draw_picture.grapic_data_struct[1].layer=5;
	draw_picture.grapic_data_struct[1].color=4;
	draw_picture.grapic_data_struct[1].start_angle=0;
	draw_picture.grapic_data_struct[1].end_angle=0;
	draw_picture.grapic_data_struct[1].width=6;
	draw_picture.grapic_data_struct[1].start_x=st_X+4;
	draw_picture.grapic_data_struct[1].start_y=st_Y-29;
	draw_picture.grapic_data_struct[1].radius=0;
	draw_picture.grapic_data_struct[1].end_x=st_X+16;
	draw_picture.grapic_data_struct[1].end_y=st_Y-41;//Q
	
	draw_picture.grapic_data_struct[2].graphic_name[0]=0;
	draw_picture.grapic_data_struct[2].graphic_name[1]=1;
	draw_picture.grapic_data_struct[2].graphic_name[2]=7;
	draw_picture.grapic_data_struct[2].operate_tpye=1;
	draw_picture.grapic_data_struct[2].graphic_tpye=1;
	draw_picture.grapic_data_struct[2].layer=4;
	draw_picture.grapic_data_struct[2].color=2;
	draw_picture.grapic_data_struct[2].start_angle=0;
	draw_picture.grapic_data_struct[2].end_angle=0;
	draw_picture.grapic_data_struct[2].width=2;
	draw_picture.grapic_data_struct[2].start_x=810;
	draw_picture.grapic_data_struct[2].start_y=440;
	draw_picture.grapic_data_struct[2].radius=0;
	draw_picture.grapic_data_struct[2].end_x=1110;
	draw_picture.grapic_data_struct[2].end_y=640;//自瞄

	draw_picture.grapic_data_struct[3].graphic_name[0]=0;
	draw_picture.grapic_data_struct[3].graphic_name[1]=1;
	draw_picture.grapic_data_struct[3].graphic_name[2]=8;
	draw_picture.grapic_data_struct[3].operate_tpye=1;
	draw_picture.grapic_data_struct[3].graphic_tpye=1;
	draw_picture.grapic_data_struct[3].layer=5;
	draw_picture.grapic_data_struct[3].color=4;
	draw_picture.grapic_data_struct[3].start_angle=0;
	draw_picture.grapic_data_struct[3].end_angle=0;
	draw_picture.grapic_data_struct[3].width=6;
	draw_picture.grapic_data_struct[3].start_x=st_X+44;
	draw_picture.grapic_data_struct[3].start_y=st_Y-29;
	draw_picture.grapic_data_struct[3].radius=0;
	draw_picture.grapic_data_struct[3].end_x=st_X+56;
	draw_picture.grapic_data_struct[3].end_y=st_Y-41;//E
	
	draw_picture.grapic_data_struct[4].graphic_name[0]=0;
	draw_picture.grapic_data_struct[4].graphic_name[1]=1;
	draw_picture.grapic_data_struct[4].graphic_name[2]=9;
	draw_picture.grapic_data_struct[4].operate_tpye=1;
	draw_picture.grapic_data_struct[4].graphic_tpye=1;
	draw_picture.grapic_data_struct[4].layer=5;
	draw_picture.grapic_data_struct[4].color=4;
	draw_picture.grapic_data_struct[4].start_angle=0;
	draw_picture.grapic_data_struct[4].end_angle=0;
	draw_picture.grapic_data_struct[4].width=6;
	draw_picture.grapic_data_struct[4].start_x=st_X+84;
	draw_picture.grapic_data_struct[4].start_y=st_Y-29;
	draw_picture.grapic_data_struct[4].radius=0;
	draw_picture.grapic_data_struct[4].end_x=st_X+96;
	draw_picture.grapic_data_struct[4].end_y=st_Y-41;//R
	
	draw_picture.grapic_data_struct[5].graphic_name[0]=0;
	draw_picture.grapic_data_struct[5].graphic_name[1]=2;
	draw_picture.grapic_data_struct[5].graphic_name[2]=0;
	draw_picture.grapic_data_struct[5].operate_tpye=1;
	draw_picture.grapic_data_struct[5].graphic_tpye=1;
	draw_picture.grapic_data_struct[5].layer=5;
	draw_picture.grapic_data_struct[5].color=4;
	draw_picture.grapic_data_struct[5].start_angle=0;
	draw_picture.grapic_data_struct[5].end_angle=0;
	draw_picture.grapic_data_struct[5].width=6;
	draw_picture.grapic_data_struct[5].start_x=st_X+124;
	draw_picture.grapic_data_struct[5].start_y=st_Y-29;
	draw_picture.grapic_data_struct[5].radius=0;
	draw_picture.grapic_data_struct[5].end_x=st_X+136;
	draw_picture.grapic_data_struct[5].end_y=st_Y-41;//V
	
	draw_picture.grapic_data_struct[6].graphic_name[0]=0;
	draw_picture.grapic_data_struct[6].graphic_name[1]=2;
	draw_picture.grapic_data_struct[6].graphic_name[2]=1;
	draw_picture.grapic_data_struct[6].operate_tpye=1;
	draw_picture.grapic_data_struct[6].graphic_tpye=1;
	draw_picture.grapic_data_struct[6].layer=5;
	draw_picture.grapic_data_struct[6].color=4;
	draw_picture.grapic_data_struct[6].start_angle=0;
	draw_picture.grapic_data_struct[6].end_angle=0;
	draw_picture.grapic_data_struct[6].width=6;
	draw_picture.grapic_data_struct[6].start_x=st_X+164;
	draw_picture.grapic_data_struct[6].start_y=st_Y-29;
	draw_picture.grapic_data_struct[6].radius=0;
	draw_picture.grapic_data_struct[6].end_x=st_X+176;
	draw_picture.grapic_data_struct[6].end_y=st_Y-41;//SH
	
	memcpy(Show_Pack+HEADER_LEN+sizeof(Picture_Header),(uint8_t *)&draw_picture,sizeof(draw_picture));
	Append_CRC16_Check_Sum_p(Show_Pack,sizeof(draw_picture)+HEADER_LEN+sizeof(Picture_Header)+CRC_LEN);
	
	HAL_UART_Transmit(huart,Show_Pack,sizeof(draw_picture)+HEADER_LEN+sizeof(Picture_Header)+CRC_LEN,0xff);
	
	
	delay(34);
	
	static ext_client_custom_character_t draw_char;
	static uint8_t Show_Pack2[100] = {0};
	picture_data_head Picture_Header2;
	ext_data_frame_header_t header2;
	
	header2.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header2.DATA_LENGTH=DCHAR_LEN;
	header2.SEQ=0;
	memcpy(Show_Pack2,(uint8_t *)&header2,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack2,HEADER_LEN);
	
	Picture_Header2.Cmd_Id=0x0301;
	Picture_Header2.Student_Interactive_Header_Data.data_cmd_id=DCHAR_ID;
	Picture_Header2.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header2.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack2+HEADER_LEN,(uint8_t *)&Picture_Header2,sizeof(Picture_Header2));
	
	draw_char.grapic_data_struct.graphic_name[0]=0;
	draw_char.grapic_data_struct.graphic_name[1]=2;
	draw_char.grapic_data_struct.graphic_name[2]=2;
	draw_char.grapic_data_struct.operate_tpye=1;
	draw_char.grapic_data_struct.graphic_tpye=7;
	draw_char.grapic_data_struct.layer=3;
	draw_char.grapic_data_struct.color=1;
	draw_char.grapic_data_struct.start_angle=10;
	draw_char.grapic_data_struct.end_angle=5;
	draw_char.grapic_data_struct.width=2;
	draw_char.grapic_data_struct.start_x=sc_X+195;
	draw_char.grapic_data_struct.start_y=sc_Y;
	draw_char.grapic_data_struct.radius=0;
	draw_char.grapic_data_struct.end_x=0;
	draw_char.grapic_data_struct.end_y=0;
	char data[5]={0};
	sprintf(data,"%d",supercap);
	memcpy(draw_char.data,(uint8_t *)&data,5);
	memcpy(Show_Pack2+HEADER_LEN+sizeof(Picture_Header2),(uint8_t *)&draw_char,sizeof(draw_char));
	Append_CRC16_Check_Sum_p(Show_Pack2,sizeof(draw_char)+HEADER_LEN+sizeof(Picture_Header2)+CRC_LEN);
	
	HAL_UART_Transmit(huart,Show_Pack2,sizeof(draw_char)+HEADER_LEN+sizeof(Picture_Header2)+CRC_LEN,0xff);
	

}

void flash_picture(UART_HandleTypeDef  *huart,uint16_t robot_id,uint16_t supercap,uint8_t keyboard)
{
	float supercap_max=24;
	float p_sc=0;
	p_sc=((float)supercap)/supercap_max;
	uint8_t color_sc;
	if(p_sc>=0.7)
		color_sc=2;
	else if(p_sc>=0.5)
		color_sc=1;
	else if(p_sc>=0)
		color_sc=4;
	
	uint8_t qc,ec,rc,vc,bc;
	
	if(keyboard%2==1)
		qc=2;
	else 
		qc=4;
	keyboard=keyboard>>1;
	
	if(keyboard%2==1)
		ec=2;
	else 
		ec=4;
	keyboard=keyboard>>1;
	
	if(keyboard%2==1)
		rc=2;
	else 
		rc=4;
	keyboard=keyboard>>1;
	
	if(keyboard%2==1)
		vc=2;
	else 
		vc=4;
	keyboard=keyboard>>1;
	
	if(keyboard%2==1)
		bc=2;
	else 
		bc=4;
	keyboard=keyboard>>1;
	
	
	static ext_client_custom_graphic_seven_t draw_picture;
	static uint8_t Show_Pack[150] = {0};
	picture_data_head Picture_Header;
	ext_data_frame_header_t header;
	
	header.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header.DATA_LENGTH=D7_LEN;
	header.SEQ=0;
	memcpy(Show_Pack,(uint8_t *)&header,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack,HEADER_LEN);
	
	Picture_Header.Cmd_Id=0x0301;
	Picture_Header.Student_Interactive_Header_Data.data_cmd_id=D7_ID;
	Picture_Header.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack+HEADER_LEN,(uint8_t *)&Picture_Header,sizeof(Picture_Header));
	
	
	draw_picture.grapic_data_struct[0].graphic_name[0]=0;
	draw_picture.grapic_data_struct[0].graphic_name[1]=1;
	draw_picture.grapic_data_struct[0].graphic_name[2]=5;
	draw_picture.grapic_data_struct[0].operate_tpye=2;
	draw_picture.grapic_data_struct[0].graphic_tpye=4;
	draw_picture.grapic_data_struct[0].layer=3;
	draw_picture.grapic_data_struct[0].color=color_sc;
	draw_picture.grapic_data_struct[0].start_angle=0;
	draw_picture.grapic_data_struct[0].end_angle=360*p_sc;
	draw_picture.grapic_data_struct[0].width=5;
	draw_picture.grapic_data_struct[0].start_x=sc_X+200;
	draw_picture.grapic_data_struct[0].start_y=sc_Y-5;
	draw_picture.grapic_data_struct[0].radius=0;
	draw_picture.grapic_data_struct[0].end_x=30;
	draw_picture.grapic_data_struct[0].end_y=30;//电容显示
	
	draw_picture.grapic_data_struct[1].graphic_name[0]=0;
	draw_picture.grapic_data_struct[1].graphic_name[1]=1;
	draw_picture.grapic_data_struct[1].graphic_name[2]=6;
	draw_picture.grapic_data_struct[1].operate_tpye=2;
	draw_picture.grapic_data_struct[1].graphic_tpye=1;
	draw_picture.grapic_data_struct[1].layer=5;
	draw_picture.grapic_data_struct[1].color=qc;
	draw_picture.grapic_data_struct[1].start_angle=0;
	draw_picture.grapic_data_struct[1].end_angle=0;
	draw_picture.grapic_data_struct[1].width=8;
	draw_picture.grapic_data_struct[1].start_x=st_X+4;
	draw_picture.grapic_data_struct[1].start_y=st_Y-29;
	draw_picture.grapic_data_struct[1].radius=0;
	draw_picture.grapic_data_struct[1].end_x=st_X+16;
	draw_picture.grapic_data_struct[1].end_y=st_Y-41;//Q
	
	draw_picture.grapic_data_struct[2].graphic_name[0]=0;
	draw_picture.grapic_data_struct[2].graphic_name[1]=1;
	draw_picture.grapic_data_struct[2].graphic_name[2]=7;
	draw_picture.grapic_data_struct[2].operate_tpye=2;
	draw_picture.grapic_data_struct[2].graphic_tpye=1;
	draw_picture.grapic_data_struct[2].layer=4;
	draw_picture.grapic_data_struct[2].color=2;
	draw_picture.grapic_data_struct[2].start_angle=0;
	draw_picture.grapic_data_struct[2].end_angle=0;
	draw_picture.grapic_data_struct[2].width=2;
	draw_picture.grapic_data_struct[2].start_x=0;
	draw_picture.grapic_data_struct[2].start_y=0;
	draw_picture.grapic_data_struct[2].radius=0;
	draw_picture.grapic_data_struct[2].end_x=0;
	draw_picture.grapic_data_struct[2].end_y=0;//自瞄

	draw_picture.grapic_data_struct[3].graphic_name[0]=0;
	draw_picture.grapic_data_struct[3].graphic_name[1]=1;
	draw_picture.grapic_data_struct[3].graphic_name[2]=8;
	draw_picture.grapic_data_struct[3].operate_tpye=2;
	draw_picture.grapic_data_struct[3].graphic_tpye=1;
	draw_picture.grapic_data_struct[3].layer=5;
	draw_picture.grapic_data_struct[3].color=ec;
	draw_picture.grapic_data_struct[3].start_angle=0;
	draw_picture.grapic_data_struct[3].end_angle=0;
	draw_picture.grapic_data_struct[3].width=8;
	draw_picture.grapic_data_struct[3].start_x=st_X+44;
	draw_picture.grapic_data_struct[3].start_y=st_Y-29;
	draw_picture.grapic_data_struct[3].radius=0;
	draw_picture.grapic_data_struct[3].end_x=st_X+56;
	draw_picture.grapic_data_struct[3].end_y=st_Y-41;//E
	
	draw_picture.grapic_data_struct[4].graphic_name[0]=0;
	draw_picture.grapic_data_struct[4].graphic_name[1]=1;
	draw_picture.grapic_data_struct[4].graphic_name[2]=9;
	draw_picture.grapic_data_struct[4].operate_tpye=2;
	draw_picture.grapic_data_struct[4].graphic_tpye=1;
	draw_picture.grapic_data_struct[4].layer=5;
	draw_picture.grapic_data_struct[4].color=rc;
	draw_picture.grapic_data_struct[4].start_angle=0;
	draw_picture.grapic_data_struct[4].end_angle=0;
	draw_picture.grapic_data_struct[4].width=8;
	draw_picture.grapic_data_struct[4].start_x=st_X+84;
	draw_picture.grapic_data_struct[4].start_y=st_Y-29;
	draw_picture.grapic_data_struct[4].radius=0;
	draw_picture.grapic_data_struct[4].end_x=st_X+96;
	draw_picture.grapic_data_struct[4].end_y=st_Y-41;//R
	
	draw_picture.grapic_data_struct[5].graphic_name[0]=0;
	draw_picture.grapic_data_struct[5].graphic_name[1]=2;
	draw_picture.grapic_data_struct[5].graphic_name[2]=0;
	draw_picture.grapic_data_struct[5].operate_tpye=2;
	draw_picture.grapic_data_struct[5].graphic_tpye=1;
	draw_picture.grapic_data_struct[5].layer=5;
	draw_picture.grapic_data_struct[5].color=vc;
	draw_picture.grapic_data_struct[5].start_angle=0;
	draw_picture.grapic_data_struct[5].end_angle=0;
	draw_picture.grapic_data_struct[5].width=8;
	draw_picture.grapic_data_struct[5].start_x=st_X+124;
	draw_picture.grapic_data_struct[5].start_y=st_Y-29;
	draw_picture.grapic_data_struct[5].radius=0;
	draw_picture.grapic_data_struct[5].end_x=st_X+136;
	draw_picture.grapic_data_struct[5].end_y=st_Y-41;//V
	
	draw_picture.grapic_data_struct[6].graphic_name[0]=0;
	draw_picture.grapic_data_struct[6].graphic_name[1]=2;
	draw_picture.grapic_data_struct[6].graphic_name[2]=1;
	draw_picture.grapic_data_struct[6].operate_tpye=2;
	draw_picture.grapic_data_struct[6].graphic_tpye=1;
	draw_picture.grapic_data_struct[6].layer=5;
	draw_picture.grapic_data_struct[6].color=bc;
	draw_picture.grapic_data_struct[6].start_angle=0;
	draw_picture.grapic_data_struct[6].end_angle=0;
	draw_picture.grapic_data_struct[6].width=8;
	draw_picture.grapic_data_struct[6].start_x=st_X+164;
	draw_picture.grapic_data_struct[6].start_y=st_Y-29;
	draw_picture.grapic_data_struct[6].radius=0;
	draw_picture.grapic_data_struct[6].end_x=st_X+176;
	draw_picture.grapic_data_struct[6].end_y=st_Y-41;//SH
	
	memcpy(Show_Pack+HEADER_LEN+sizeof(Picture_Header),(uint8_t *)&draw_picture,sizeof(draw_picture));
	Append_CRC16_Check_Sum_p(Show_Pack,sizeof(draw_picture)+HEADER_LEN+sizeof(Picture_Header)+CRC_LEN);
	
	HAL_UART_Transmit(huart,Show_Pack,sizeof(draw_picture)+HEADER_LEN+sizeof(Picture_Header)+CRC_LEN,0xff);
	
	
	delay(34);
	
	static ext_client_custom_character_t draw_char;
	static uint8_t Show_Pack2[100] = {0};
	picture_data_head Picture_Header2;
	ext_data_frame_header_t header2;
	
	header2.SOF=DATA_FRAME_HEADER_SOF_DATA;
	header2.DATA_LENGTH=DCHAR_LEN;
	header2.SEQ=0;
	memcpy(Show_Pack2,(uint8_t *)&header2,HEADER_LEN);
	Append_CRC8_Check_Sum_p(Show_Pack2,HEADER_LEN);
	
	Picture_Header2.Cmd_Id=0x0301;
	Picture_Header2.Student_Interactive_Header_Data.data_cmd_id=DCHAR_ID;
	Picture_Header2.Student_Interactive_Header_Data.sender_ID=robot_id;
	Picture_Header2.Student_Interactive_Header_Data.receiver_ID=robot_id+0x0100;
	memcpy(Show_Pack2+HEADER_LEN,(uint8_t *)&Picture_Header2,sizeof(Picture_Header2));
	
	draw_char.grapic_data_struct.graphic_name[0]=0;
	draw_char.grapic_data_struct.graphic_name[1]=1;
	draw_char.grapic_data_struct.graphic_name[2]=3;
	draw_char.grapic_data_struct.operate_tpye=2;
	draw_char.grapic_data_struct.graphic_tpye=7;
	draw_char.grapic_data_struct.layer=3;
	draw_char.grapic_data_struct.color=1;
	draw_char.grapic_data_struct.start_angle=10;
	draw_char.grapic_data_struct.end_angle=5;
	draw_char.grapic_data_struct.width=2;
	draw_char.grapic_data_struct.start_x=sc_X+195;
	draw_char.grapic_data_struct.start_y=sc_Y;
	draw_char.grapic_data_struct.radius=0;
	draw_char.grapic_data_struct.end_x=0;
	draw_char.grapic_data_struct.end_y=0;
	char data[5]={0};
	sprintf(data,"%d",supercap);
	memcpy(draw_char.data,(uint8_t *)&data,5);
	memcpy(Show_Pack2+HEADER_LEN+sizeof(Picture_Header2),(uint8_t *)&draw_char,sizeof(draw_char));
	Append_CRC16_Check_Sum_p(Show_Pack2,sizeof(draw_char)+HEADER_LEN+sizeof(Picture_Header2)+CRC_LEN);
	
	HAL_UART_Transmit(huart,Show_Pack2,sizeof(draw_char)+HEADER_LEN+sizeof(Picture_Header2)+CRC_LEN,0xff);
	

}






