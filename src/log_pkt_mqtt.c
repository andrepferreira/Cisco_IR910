/*
Description:
	Collect packets of the Lora interface and send them out on the MQTT bus

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Andre Ferreira
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
	#define _XOPEN_SOURCE 600
#else
	#define _XOPEN_SOURCE 500
#endif

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf sprintf fopen fputs */

#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */
#include <time.h>		/* time clock_gettime strftime gmtime clock_nanosleep*/
#include <unistd.h>		/* getopt access */
#include <stdlib.h>		/* atoi */
#include <math.h>		/* Maths functions */

#include "parson.h"
#include "loragw_hal.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#define MSG(args...)	fprintf(stderr,"mqtt_pkt_logger: " args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* configuration variables needed by the application  */
uint64_t lgwm = 0; /* LoRa gateway MAC address */
char lgwm_str[17];

/* clock management */
time_t now_time;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

int parse_SX1301_configuration(const char * conf_file);

int parse_gateway_configuration(const char * conf_file);

void usage (void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
	if (sigio == SIGQUIT) {
		quit_sig = 1;;
	} else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
		exit_sig = 1;
	}
}

int parse_SX1301_configuration(const char * conf_file) {
	int i;
	const char conf_obj[] = "SX1301_conf";
	char param_name[32]; /* used to generate variable parameter names */
	struct lgw_conf_rxrf_s rfconf;
	struct lgw_conf_rxif_s ifconf;
	JSON_Value *root_val;
	JSON_Object *root = NULL;
	JSON_Object *conf = NULL;
	JSON_Value *val;
	uint32_t sf, bw;
	
	/* try to parse JSON */
	root_val = json_parse_file_with_comments(conf_file);
	root = json_value_get_object(root_val);
	if (root == NULL) {
		MSG("ERROR: %s id not a valid JSON file\n", conf_file);
		exit(EXIT_FAILURE);
	}
	conf = json_object_get_object(root, conf_obj);
	if (conf == NULL) {
		MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj);
		return -1;
	} else {
		MSG("INFO: %s does contain a JSON object named %s, parsing SX1301 parameters\n", conf_file, conf_obj);
	}
	
	/* set configuration for RF chains */
	for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
		memset(&rfconf, 0, sizeof(rfconf)); /* initialize configuration structure */
		sprintf(param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
		val = json_object_get_value(conf, param_name); /* fetch value (if possible) */
		if (json_value_get_type(val) != JSONObject) {
			MSG("INFO: no configuration for radio %i\n", i);
			continue;
		}
		/* there is an object to configure that radio, let's parse it */
		sprintf(param_name, "radio_%i.enable", i);
		val = json_object_dotget_value(conf, param_name);
		if (json_value_get_type(val) == JSONBoolean) {
			rfconf.enable = (bool)json_value_get_boolean(val);
		} else {
			rfconf.enable = false;
		}
		if (rfconf.enable == false) { /* radio disabled, nothing else to parse */
			MSG("INFO: radio %i disabled\n", i);
		} else  { /* radio enabled, will parse the other parameters */
			sprintf(param_name, "radio_%i.freq", i);
			rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf, param_name);
			MSG("INFO: radio %i enabled, center frequency %u\n", i, rfconf.freq_hz);
		}
		/* all parameters parsed, submitting configuration to the HAL */
		if (lgw_rxrf_setconf(i, rfconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for radio %i\n", i);
		}
	}
	
	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	for (i = 0; i < LGW_MULTI_NB; ++i) {
		memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
		sprintf(param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
		val = json_object_get_value(conf, param_name); /* fetch value (if possible) */
		if (json_value_get_type(val) != JSONObject) {
			MSG("INFO: no configuration for LoRa multi-SF channel %i\n", i);
			continue;
		}
		/* there is an object to configure that LoRa multi-SF channel, let's parse it */
		sprintf(param_name, "chan_multiSF_%i.enable", i);
		val = json_object_dotget_value(conf, param_name);
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}
		if (ifconf.enable == false) { /* LoRa multi-SF channel disabled, nothing else to parse */
			MSG("INFO: LoRa multi-SF channel %i disabled\n", i);
		} else  { /* LoRa multi-SF channel enabled, will parse the other parameters */
			sprintf(param_name, "chan_multiSF_%i.radio", i);
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, param_name);
			sprintf(param_name, "chan_multiSF_%i.if", i);
			ifconf.freq_hz = (int32_t)json_object_dotget_number(conf, param_name);
			// TODO: handle individual SF enabling and disabling (spread_factor)
			MSG("INFO: LoRa multi-SF channel %i enabled, radio %i selected, IF %i Hz, 125 kHz bandwidth, SF 7 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
		}
		/* all parameters parsed, submitting configuration to the HAL */
		if (lgw_rxif_setconf(i, ifconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for LoRa multi-SF channel %i\n", i);
		}
	}
	
	/* set configuration for LoRa standard channel */
	memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
	val = json_object_get_value(conf, "chan_Lora_std"); /* fetch value (if possible) */
	if (json_value_get_type(val) != JSONObject) {
		MSG("INFO: no configuration for LoRa standard channel\n");
	} else {
		val = json_object_dotget_value(conf, "chan_Lora_std.enable");
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}
		if (ifconf.enable == false) {
			MSG("INFO: LoRa standard channel %i disabled\n", i);
		} else  {
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.radio");
			ifconf.freq_hz = (int32_t)json_object_dotget_number(conf, "chan_Lora_std.if");
			bw = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.bandwidth");
			switch(bw) {
				case 500000: ifconf.bandwidth = BW_500KHZ; break;
				case 250000: ifconf.bandwidth = BW_250KHZ; break;
				case 125000: ifconf.bandwidth = BW_125KHZ; break;
				default: ifconf.bandwidth = BW_UNDEFINED;
			}
			sf = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.spread_factor");
			switch(sf) {
				case  7: ifconf.datarate = DR_LORA_SF7;  break;
				case  8: ifconf.datarate = DR_LORA_SF8;  break;
				case  9: ifconf.datarate = DR_LORA_SF9;  break;
				case 10: ifconf.datarate = DR_LORA_SF10; break;
				case 11: ifconf.datarate = DR_LORA_SF11; break;
				case 12: ifconf.datarate = DR_LORA_SF12; break;
				default: ifconf.datarate = DR_UNDEFINED;
			}
			MSG("INFO: LoRa standard channel enabled, radio %i selected, IF %i Hz, %u Hz bandwidth, SF %u\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf);
		}
		if (lgw_rxif_setconf(8, ifconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for LoRa standard channel\n");
		}
	}
	
	/* set configuration for FSK channel */
	memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
	val = json_object_get_value(conf, "chan_FSK"); /* fetch value (if possible) */
	if (json_value_get_type(val) != JSONObject) {
		MSG("INFO: no configuration for FSK channel\n");
	} else {
		val = json_object_dotget_value(conf, "chan_FSK.enable");
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}
		if (ifconf.enable == false) {
			MSG("INFO: FSK channel %i disabled\n", i);
		} else  {
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, "chan_FSK.radio");
			ifconf.freq_hz = (int32_t)json_object_dotget_number(conf, "chan_FSK.if");
			bw = (uint32_t)json_object_dotget_number(conf, "chan_FSK.bandwidth");
			if      (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
			else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
			else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
			else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
			else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
			else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
			else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
			else ifconf.bandwidth = BW_UNDEFINED;
			ifconf.datarate = (uint32_t)json_object_dotget_number(conf, "chan_FSK.datarate");
			MSG("INFO: FSK channel enabled, radio %i selected, IF %i Hz, %u Hz bandwidth, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
		}
		if (lgw_rxif_setconf(9, ifconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for FSK channel\n");
		}
	}
	json_value_free(root_val);
	return 0;
}

int parse_gateway_configuration(const char * conf_file) {
	const char conf_obj[] = "gateway_conf";
	JSON_Value *root_val;
	JSON_Object *root = NULL;
	JSON_Object *conf = NULL;
	unsigned long long ull = 0;
	
	/* try to parse JSON */
	root_val = json_parse_file_with_comments(conf_file);
	root = json_value_get_object(root_val);
	if (root == NULL) {
		MSG("ERROR: %s id not a valid JSON file\n", conf_file);
		exit(EXIT_FAILURE);
	}
	conf = json_object_get_object(root, conf_obj);
	if (conf == NULL) {
		MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj);
		return -1;
	} else {
		MSG("INFO: %s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj);
	}
	
	/* getting network parameters (only those necessary for the packet logger) */
	sscanf(json_object_dotget_string(conf, "gateway_ID"), "%llx", &ull);
	lgwm = ull;
	MSG("INFO: gateway MAC address is configured to %016llX\n", ull);
	
	json_value_free(root_val);
	return 0;
}

/* describe command line options */
void usage(void) {
	printf("*** Library version information ***\n%s\n\n", lgw_version_info());
	printf( "Available options:\n");
	printf( " -h print this help\n");
}

/* Andre's functions-----------------------------------------------------------*/

void send_mqtt_float (char* srv_add, char* username, char* password, char* topic, float value)
{
/* Code to send RSSI to MQTT server
	 * This snippit creates a string of the command to send a value to a
	 * defined topic on the MQTT server.
	 * Command is run as a system() command directly on the OS.
	 * Needs mosquitto_pub as OS command to work
	 */
	char command[256];
	char res[30];
		strcpy (command, "mosquitto_pub -h ");
		strcat (command, srv_add);
		strcat (command, " -u ");
		strcat (command, username);
		strcat (command, " -P ");
		strcat (command, password);
		strcat (command, " -t ");
		strcat (command, topic);
		strcat (command, " -m ");


		//Convert float to string
		snprintf(res,30,"%f",value);

		strcat(command, res);
		//printf("Command is %s\n", command);
		system (command);
}

void send_mqtt_string (char* srv_add, char* username, char* password, char* topic, char* value)
{
/* Code to send RSSI to MQTT server
	 * This snippit creates a string of the command to send a value to a
	 * defined topic on the MQTT server.
	 * Command is run as a system() command directly on the OS.
	 * Needs mosquitto_pub as OS command to work
	 */
	char command[256];
	char res[30];
		strcpy (command, "mosquitto_pub -h ");
		strcat (command, srv_add);
		strcat (command, " -u ");
		strcat (command, username);
		strcat (command, " -P ");
		strcat (command, password);
		strcat (command, " -t ");
		strcat (command, topic);
		strcat (command, " -m ");


		//Convert float to string
		snprintf(res,30,"%s",value);

		strcat(command, res);
		//printf("Command is %s\n", command);
		system (command);
}

/* -----------------------------------------------end of Andre's stuff---*/


/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
	int i, j; /* loop and temporary variables */
	struct timespec sleep_time = {0, 3000000}; /* 3 ms */
	
	/* configuration file related */
	const char global_conf_fname[] = "global_conf.json"; /* contain global (typ. network-wide) configuration */
	const char local_conf_fname[] = "local_conf.json"; /* contain node specific configuration, overwrite global parameters for parameters that are defined in both */
	const char debug_conf_fname[] = "debug_conf.json"; /* if present, all other configuration files are ignored */
	
	/* allocate memory for packet fetching and processing */
	struct lgw_pkt_rx_s rxpkt[16]; /* array containing up to 16 inbound packets metadata */
	struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
	int nb_pkt;
	
	/* local timestamp variables until we get accurate GPS time */
	struct timespec fetch_time;
	char fetch_timestamp[30];
	struct tm * x;
	
	/* parse command line options */
	while ((i = getopt (argc, argv, "hr:")) != -1) {
		switch (i) {
			case 'h':
				usage();
				return EXIT_FAILURE;
				break;
			
			default:
				MSG("ERROR: argument parsing use -h option for help\n");
				usage();
				return EXIT_FAILURE;
		}
	}
	
	/* configure signal handling */
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = sig_handler;
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	
	/* configuration files management */
	if (access(debug_conf_fname, R_OK) == 0) {
	/* if there is a debug conf, parse only the debug conf */
		MSG("INFO: found debug configuration file %s, other configuration files will be ignored\n", debug_conf_fname);
		parse_SX1301_configuration(debug_conf_fname);
		parse_gateway_configuration(debug_conf_fname);
	} else if (access(global_conf_fname, R_OK) == 0) {
	/* if there is a global conf, parse it and then try to parse local conf  */
		MSG("INFO: found global configuration file %s, trying to parse it\n", global_conf_fname);
		parse_SX1301_configuration(global_conf_fname);
		parse_gateway_configuration(global_conf_fname);
		if (access(local_conf_fname, R_OK) == 0) {
			MSG("INFO: found local configuration file %s, trying to parse it\n", local_conf_fname);
			parse_SX1301_configuration(local_conf_fname);
			parse_gateway_configuration(local_conf_fname);
		}
	} else if (access(local_conf_fname, R_OK) == 0) {
	/* if there is only a local conf, parse it and that's all */
		MSG("INFO: found local configuration file %s, trying to parse it\n", local_conf_fname);
		parse_SX1301_configuration(local_conf_fname);
		parse_gateway_configuration(local_conf_fname);
	} else {
		MSG("ERROR: failed to find any configuration file named %s, %s or %s\n", global_conf_fname, local_conf_fname, debug_conf_fname);
		return EXIT_FAILURE;
	}
	
	/* starting the concentrator */
	i = lgw_start();
	if (i == LGW_HAL_SUCCESS) {
		MSG("INFO: concentrator started, packet can now be received\n");
	} else {
		MSG("ERROR: failed to start the concentrator\n");
		return EXIT_FAILURE;
	}
	
	/* transform the MAC address into a string */
	sprintf(lgwm_str, "%08X%08X", (uint32_t)(lgwm >> 32), (uint32_t)(lgwm & 0xFFFFFFFF));
	

	/* main loop */
	while ((quit_sig != 1) && (exit_sig != 1)) {
		/* fetch packets */
		nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
		if (nb_pkt == LGW_HAL_ERROR) {
			MSG("ERROR: failed packet fetch, exiting\n");
			return EXIT_FAILURE;
		} else if (nb_pkt == 0) {
			clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
		} else {
			/* local timestamp generation until we get accurate GPS time */
			clock_gettime(CLOCK_REALTIME, &fetch_time);
			x = gmtime(&(fetch_time.tv_sec));
			sprintf(fetch_timestamp,"%04i-%02i-%02i %02i:%02i:%02i.%03liZ",(x->tm_year)+1900,(x->tm_mon)+1,x->tm_mday,x->tm_hour,x->tm_min,x->tm_sec,(fetch_time.tv_nsec)/1000000); /* ISO 8601 format */

		}
		
		/* Collect packets */
		for (i=0; i < nb_pkt; ++i) {
			p = &rxpkt[i];
			

			/* writing UTC timestamp*/
			printf ("Time: %s\n", fetch_timestamp);
			// TODO: replace with GPS time when available
			
			/* writing internal clock */
			printf("Concentrator count: %10u\n", p->count_us);
			
			/* writing RX frequency */
			printf ("Freq: %10u\n", p->freq_hz);
			
			/* writing RF chain */
			printf("RF Chain: %u\n", p->rf_chain);
			
			/* writing RX modem/IF chain */
			printf("RX/IF Chain: %2d\n", p->if_chain);
			
			/* Check packet CRC status */
			switch(p->status) {
				case STAT_CRC_OK:	printf("CRC_OK\n"); break;
				case STAT_CRC_BAD:	printf("CRC_BAD\n"); break;
				case STAT_NO_CRC:	printf("NO_CRC\n"); break;
				case STAT_UNDEFINED:printf("UNDEF\n"); break;
				default: printf("ERR ");
			}
			
			//If CRC is bad stop processing
			if (p->status == STAT_CRC_BAD) {
				printf ("Bad packet\n");
				break;
			}


			/* writing payload size */
			printf("Payload size: %3u\n", p->size);
			
			/* writing modulation */
			switch(p->modulation) {
				case MOD_LORA:	printf("LORA\n"); break;
				case MOD_FSK:	printf("FSK\n"); break;
				default: printf("ERR\n");
			}
			
			/* writing bandwidth
			switch(p->bandwidth) {
				case BW_500KHZ:	fputs("500000,", log_file); break;
				case BW_250KHZ:	fputs("250000,", log_file); break;
				case BW_125KHZ:	fputs("125000,", log_file); break;
				case BW_62K5HZ:	fputs("62500 ,", log_file); break;
				case BW_31K2HZ:	fputs("31200 ,", log_file); break;
				case BW_15K6HZ:	fputs("15600 ,", log_file); break;
				case BW_7K8HZ:	fputs("7800  ,", log_file); break;
				case BW_UNDEFINED: fputs("0     ,", log_file); break;
				default: fputs("-1    ,", log_file);
			} */
			
			/* writing datarate
			if (p->modulation == MOD_LORA) {
				switch (p->datarate) {
					case DR_LORA_SF7:	fputs("\"SF7\"   ,", log_file); break;
					case DR_LORA_SF8:	fputs("\"SF8\"   ,", log_file); break;
					case DR_LORA_SF9:	fputs("\"SF9\"   ,", log_file); break;
					case DR_LORA_SF10:	fputs("\"SF10\"  ,", log_file); break;
					case DR_LORA_SF11:	fputs("\"SF11\"  ,", log_file); break;
					case DR_LORA_SF12:	fputs("\"SF12\"  ,", log_file); break;
					default: fputs("\"ERR\"   ,", log_file);
				}
			} else if (p->modulation == MOD_FSK) {
				fprintf(log_file, "\"%6u\",", p->datarate);
			} else {
				fputs("\"ERR\"   ,", log_file);
			} */
			
			/* writing coderate
			switch (p->coderate) {
				case CR_LORA_4_5:	fputs("\"4/5\",", log_file); break;
				case CR_LORA_4_6:	fputs("\"2/3\",", log_file); break;
				case CR_LORA_4_7:	fputs("\"4/7\",", log_file); break;
				case CR_LORA_4_8:	fputs("\"1/2\",", log_file); break;
				case CR_UNDEFINED:	fputs("\"\"   ,", log_file); break;
				default: fputs("\"ERR\",", log_file);
			} */
			
			/* writing packet RSSI */
			//printf("RSSI: %+.0f\n", p->rssi);


			// Send the RSSI value to the queue
			//send_mqtt_float ("10.12.14.68", "gateway", "password", "ir910/lora/rssi", p->rssi);


			/* writing packet average SNR */
			//printf("SNR: %+5.1f\n", p->snr);
			
			// Send the SNR value to the queue
			//send_mqtt_float ("10.12.14.68", "gateway", "password", "ir910/lora/snr", p->snr);

			/* writing hex-encoded payload
			for (j = 0; j < p->size; ++j) {
				printf("%02X", p->payload[j]);
			}
			printf("\n"); // CR to make things look pretty
			*/

			/* Code to break payload into array of strings
			 * Payload must be delimited with #
			 * Result is array (pay_array[]) of 257 char strings
			 * Number of arrays depend on the payload and is arbitrary
			 */

			char* string = p->payload;
			char tmp[10];

			//Convert float rssi and snr to string and add to the end of the payload
			snprintf(tmp, 10,"#%f",p->rssi);
			strcat (string, tmp);
			snprintf(tmp, 10,"#%f",p->snr);
			strcat (string, tmp);

			char** pay_array = (char**)malloc(257*sizeof(char*));

			memset(pay_array, 0, sizeof(char*)*257);

			// What is the delimiter? Here it is #
			char* curToken = strtok(string, "#");

			for (int i = 0; curToken != NULL; ++i)
			{
				pay_array[i] = strdup(curToken);
				curToken = strtok(NULL, "#");
			}

			/* Print out the array of payload parts and send to MQTT server
			 	 Skip index 0 array as it seems to be junk*/
			char topic[40];
			for (int n=1; pay_array[n] != NULL; ++n)
			{
				printf ("Payload [%d] is: %s\n", n, pay_array[n]);
				// Make the topic. Use node ID and index as sub-topic
				// Note, this assumes node ID is at index 1
				snprintf(topic,40,"ir910/lora/%s/%d", pay_array[1],n);
				send_mqtt_string ("10.12.14.68", "gateway", "password", topic, pay_array[n]);
			}



			/* Andre's stuff - Send a packet back


#define		RF_CHAIN				0	// we'll use radio A only
			uint8_t status_var;
			struct lgw_pkt_tx_s txpkt; // array containing 1 outbound packet + metadata
			const uint32_t lowfreq[LGW_RF_CHAIN_NB] = LGW_RF_TX_LOWFREQ;
			const uint32_t upfreq[LGW_RF_CHAIN_NB] = LGW_RF_TX_UPFREQ;

			// application parameters
				uint32_t f_target = lowfreq[RF_CHAIN]/2 + upfreq[RF_CHAIN]/2; // target frequency
				int sf = 12; // SF10 by default
				int cr = 1; // CR1 aka 4/5 by default
				int bw = 125; // 125kHz bandwidth by default
				int pow = 14; // 14 dBm by default
				int preamb = 8; // 8 symbol preamble by default
				int pl_size = 16; // 16 bytes payload by default
				//int delay = 1000; // 1 second between packets by default
				//int repeat = -1; // by default, repeat until stopped
				bool invert = false;

			// fill-up payload and parameters
				memset(&txpkt, 0, sizeof(txpkt));
				//txpkt.freq_hz = f_target;
				txpkt.freq_hz = 868500000;
				txpkt.tx_mode = IMMEDIATE;
				txpkt.rf_chain = RF_CHAIN;
				txpkt.rf_power = pow;
				txpkt.modulation = MOD_LORA;
				txpkt.bandwidth = BW_125KHZ;
				txpkt.datarate = DR_LORA_SF12;
				txpkt.coderate = CR_LORA_4_5;
				txpkt.no_crc = 0;
				txpkt.no_header = 0;

				txpkt.invert_pol = invert;
				txpkt.preamble = preamb;
				txpkt.size = pl_size;
				strcpy((char *)txpkt.payload, "This is a return message" ); // abc.. is for padding

				printf ("Freq: %u", txpkt.freq_hz);
				printf ("Tx Mode: %u", txpkt.tx_mode);
				printf ("Chain: %u",txpkt.rf_chain);
				printf ("Tx pwr: %u", txpkt.rf_power);
				printf ("Data rate: %u", txpkt.datarate);

			printf("Sending return packet");
			i = lgw_send(txpkt); // non-blocking scheduling of TX packet
			if (i != LGW_HAL_SUCCESS) {
				printf("ERROR\n");
				return EXIT_FAILURE;
			}

			// wait for packet to finish sending
			do {
				wait_ms(5000);
				lgw_status(TX_STATUS, &status_var); // get TX status
			} while (status_var != TX_FREE);
			printf("OK\n");

			end of Andre's sending stuff
			*/

		}
		
	}
	
	if (exit_sig == 1) {
		/* clean up before leaving */
		i = lgw_stop();
		if (i == LGW_HAL_SUCCESS) {
			MSG("INFO: concentrator stopped successfully\n");
		} else {
			MSG("WARNING: failed to stop concentrator successfully\n");
		}
	}
	
	MSG("INFO: Exiting packet logger program\n");
	return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
