/*
 * File      : main.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>

#include <stdlib.h>
#include <string.h>
#include <rtthread.h>
#include <at.h>

#include <at_socket.h>		//
#include "sal_socket.h"		//

/* AT+CIFSR            Query local IP address and MAC */
int at_client_test(int argc, char **argv)
{
    at_response_t resp = RT_NULL;
    int result = 0;

    if (argc != 1)
    {
        LOG_E("at_client_test  - AT client send commands to AT server.");
        return -1;
    }

    /* ������Ӧ�ṹ�壬�������֧����Ӧ���ݳ���Ϊ 256 �ֽ�
    �������Ӧ�����û�����ʵ�������Զ��壩����Ӧ�������������ƣ���ʱʱ��Ϊ 5 �� */
    resp = at_create_resp(256, 0, rt_tick_from_millisecond(5000));
    if (resp == RT_NULL)
    {
        LOG_E("No memory for response structure!");
        return -2;
    }

    /* �رջ��Թ��� */
    at_exec_cmd(resp, "ATE0");

    /* AT  Client ���Ͳ�ѯ IP ��ַ������� AT Server ��Ӧ */
    /* ��Ӧ���ݼ���Ϣ����� resp �ṹ���� */
    result = at_exec_cmd(resp, "AT+CIFSR");
    if (result != RT_EOK)
    {
        LOG_E("AT client send commands failed or return response error!");
        goto __exit;
    }

    /* ������ѭ����ӡ���յ�����Ӧ���� */
    {
        const char *line_buffer = RT_NULL;

        LOG_D("Response buffer");
        for(rt_size_t line_num = 1; line_num <= resp->line_counts; line_num++)
        {
            if((line_buffer = at_resp_get_line(resp, line_num)) != RT_NULL)
            {
                LOG_D("line %d buffer : %s", line_num, line_buffer);
            }
            else
            {
                LOG_E("Parse line buffer error!");
            }
        }
    }
    /* ���Զ�����ʽ��sscanf ������ʽ���������ݣ��õ���Ӧ���� */
    {
        char resp_arg[AT_CMD_MAX_LEN] = { 0 };
        /* �Զ������ݽ������ʽ �����ڽ�����˫����֮���ַ�����Ϣ */
        const char * resp_expr = "%*[^\"]\"%[^\"]\"";

        LOG_D(" Parse arguments");
        /* ������Ӧ�����е�һ�����ݣ��õ���Ӧ IP ��ַ */
        if (at_resp_parse_line_args(resp, 1, resp_expr, resp_arg) == 1)
        {
            LOG_D("Station IP  : %s", resp_arg);
            memset(resp_arg, 0x00, AT_CMD_MAX_LEN);
        }
        else
        {
            LOG_E("Parse error, current line buff : %s", at_resp_get_line(resp, 4));
        }

        /* ������Ӧ�����еڶ������ݣ��õ���Ӧ MAC ��ַ */
        if (at_resp_parse_line_args(resp, 2, resp_expr, resp_arg) == 1)
        {
            LOG_D("Station MAC : %s", resp_arg);
        }
        else
        {
            LOG_E("Parse error, current line buff : %s", at_resp_get_line(resp, 5));
            goto __exit;
        }
    }
__exit:
    if(resp)
    {
        /* ɾ�� resp �ṹ�� */
        at_delete_resp(resp);
    }

    return result;
}
/* ���õ�ǰ AT �ͻ������֧�ֵ�һ�ν������ݵĳ��� */
#define AT_CLIENT_RECV_BUFF_LEN         512
int at_client_test_init(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("at_client_init <dev_name>   -- AT client initialize.\n");
        return -RT_ERROR;
    }

    at_client_init(argv[1], AT_CLIENT_RECV_BUFF_LEN);

    return RT_EOK;
}
#ifdef FINSH_USING_MSH
#include <finsh.h>
/* ��� AT Client ������� shell  */
MSH_CMD_EXPORT(at_client_test, AT client send cmd and get response);
/* ��� AT Client ��ʼ����� shell  */
MSH_CMD_EXPORT_ALIAS(at_client_test_init, at_client_init, initialize AT client);
#endif

/*
��ʼ�� 
ok! connect
receive data


*/


/*
iap�̼�����ƽ̨:

https://180.101.147.135:8843/index.html#/device
�˺ţ�modou2017
���룺Cldz2017~
*/

/*
�豸����IP	180.101.147.115
�˿� 5683
*/


int main(void){
	int sock=0;
	struct sockaddr_in sockadd={0};
	at_socket_device_init();
	
	for(;;){
		if(0!=(sock=at_socket(AF_INET,SOCK_DGRAM,0))){
			sockadd.sin_port=htons(5683);
			sockadd.sin_addr.s_addr=inet_addr("180.101.147.115");
			sockadd.sin_family=AF_INET;
			
			if(at_bind(sock,(struct sockaddr*)&sockadd,sizeof(sockadd))>=0){
				for(;;){
					at_send(sock,"hello",strlen("hello"),0);
					//rt_kprintf("System run\r\n");
					rt_thread_delay(rt_tick_from_millisecond(2000));

//					at_recv();
				}
			}else{
				rt_kprintf("----bind failed\r\n");
			}
		}else{
			rt_kprintf("----socket init failed\r\n");
		
		}
		rt_kprintf("System run\r\n");
		rt_thread_delay(rt_tick_from_millisecond(1000));				
	
	}

	
  return 0;
}