#ifndef __DATA_PROCESSING_HH
#define __DATA_PROCESSING_HH

#include <string.h>

//typedef char u8;
//数据格式 0xff,0xfe 开头 跟u8的id '\n','\r' 结尾  任意类型数据后接一字节的校验和
const static char head_char[2] = {0xff,0xfe};
const static char end_char[2]  = {'\n','\r'};

////数据处理模板
class DataProcess
{
     //数据格式 0xff,0xfe 开头 跟u8的id  '\n','\r' 结尾  任意类型数据后接一字节的校验和
    public:
			DataProcess(){
				get_head_id=get_head=get_end=false;
				rec_len=0;
				rec_head_arr[0]=rec_head_arr[1]=' ';
				rec_end_arr[0]=rec_end_arr[1]=' ';
				};
			 //获取头信息
			  bool getHeadMsg(char _ch)
				{
					if(!get_head)
						{
							rec_head_arr[0] = rec_head_arr[1];
							rec_head_arr[1] = _ch;
							if(rec_head_arr[0] == head_char[0] && rec_head_arr[1] == head_char[1]) 
							{
								get_head=true;
								rec_len=0;
							}
						}
						else if(!get_head_id)
						{
							head_id = _ch;
							get_head_id = true ;
						}
						else return  get_head_id ;
						
						return false;
				};

				void clearFlag()
				{
						rec_head_arr[0]=rec_head_arr[1]=' ';
						rec_end_arr[0]=rec_end_arr[1]=' ';
						get_head_id=get_head=get_end=false;rec_len=0;
				};
				
				//获取headid
				char headId(){return head_id;};
				//清除一些标志
				void clearFalg()
				{
					rec_head_arr[0]=rec_head_arr[1]=' ';
					rec_end_arr[0]=rec_end_arr[1]=' ';
					get_head_id=get_head=get_end=false;rec_len=0;
				};
				
				//解码 
				template<typename T>  bool dataDecode(char _ch,T *stru);
				//编码  返回字符串  _len为字符串长度
				template<typename T>  char* dataEncode(T *stru,char _head_id,int *_len);
		private:
				const static int ARRSIZE=128;
				bool get_head,get_end,get_head_id;
				char rec_head_arr[2];
				char rec_end_arr[2];
				char rec_arr[ARRSIZE];                //接收数组
				char send_arr[ARRSIZE];               //发送数组
				int rec_len;
				char head_id;
        char checkSum( char* ch,int n)
				{
					char _sum=0;
					for(int i=0;i<n;i++)
							_sum+=ch[i];
					return _sum;
				};
};


template<typename T> 
bool DataProcess::dataDecode(char _ch,T *stru)
{
	if(rec_len < sizeof(T) + sizeof(end_char) + 1) //数据  endchar 检验和
	{
		rec_arr[rec_len] = _ch;
		rec_len++;
		rec_end_arr[0] = rec_end_arr[1];
		rec_end_arr[1] = _ch;
		if(rec_end_arr[0] == end_char[0] && rec_end_arr[1] == end_char[1]) 
			get_end=true;
	}
	else  //数据满了  重来
	{
		clearFlag();
	}
	
	if(get_end)
	{
		clearFlag();
		memcpy((char *)stru,&rec_arr[0],sizeof(T));
    return (rec_arr[sizeof(T)] == checkSum(&rec_arr[0],sizeof(T)));
	}
	return false;
}

template<typename T>  
char* DataProcess::dataEncode(T *stru,char _head_id,int *_len)
{
	for(int i=0;i < sizeof(head_char) ; i++ )
	{
		send_arr[i]= head_char[i];
	}
	send_arr[ sizeof(head_char)] = _head_id;
	memcpy(&send_arr[sizeof(head_char) + 1 ],(char *)stru,sizeof(T));
	send_arr[sizeof(T) + sizeof(head_char) +1 ] = checkSum(&send_arr[sizeof(head_char) + 1],sizeof(T));
	for(int i=0;i <= sizeof(end_char) ; i++ )
	{
		send_arr[sizeof(T) + sizeof(head_char) + 2 + i]= end_char[i];
	}
	*_len = sizeof(T) + sizeof(head_char) + sizeof(end_char) +2 ;
	return send_arr;
}



#endif