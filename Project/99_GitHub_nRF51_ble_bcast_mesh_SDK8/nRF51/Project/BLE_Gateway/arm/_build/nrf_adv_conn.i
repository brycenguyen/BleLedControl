#line 1 "..\\nrf_adv_conn.c"

































 

#line 1 "..\\include\\nrf_adv_conn.h"

































 




#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"


































 









 




#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_ranges.h"


































 



















 































































 
#line 51 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_types.h"


































 








 




#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 







 




  
 







#line 37 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 208 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 272 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 50 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_types.h"


 


 


 



 
 
#line 75 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_types.h"
 


 
#line 85 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_types.h"
 



 



 




 
#line 148 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_types.h"
 

 




 




 




 



 



 


 

 
typedef struct
{ 
    unsigned char uuid128[16];  
} ble_uuid128_t;

 
typedef struct
{
    uint16_t    uuid;  
    uint8_t     type;  
} ble_uuid_t;

 






 
#line 52 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"


































 





 




#line 47 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"
#line 48 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_svc.h"


































 
 







#line 69 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_svc.h"

#line 49 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"


 


 
enum BLE_GAP_SVCS
{
  SD_BLE_GAP_ADDRESS_SET  = 0x70,   
  SD_BLE_GAP_ADDRESS_GET,                       
  SD_BLE_GAP_ADV_DATA_SET,                      
  SD_BLE_GAP_ADV_START,                         
  SD_BLE_GAP_ADV_STOP,                          
  SD_BLE_GAP_CONN_PARAM_UPDATE,                 
  SD_BLE_GAP_DISCONNECT,                        
  SD_BLE_GAP_TX_POWER_SET,                      
  SD_BLE_GAP_APPEARANCE_SET,                    
  SD_BLE_GAP_APPEARANCE_GET,                    
  SD_BLE_GAP_PPCP_SET,                          
  SD_BLE_GAP_PPCP_GET,                          
  SD_BLE_GAP_DEVICE_NAME_SET,                   
  SD_BLE_GAP_DEVICE_NAME_GET,                   
  SD_BLE_GAP_AUTHENTICATE,                      
  SD_BLE_GAP_SEC_PARAMS_REPLY,                  
  SD_BLE_GAP_AUTH_KEY_REPLY,                    
  SD_BLE_GAP_ENCRYPT,                           
  SD_BLE_GAP_SEC_INFO_REPLY,                    
  SD_BLE_GAP_CONN_SEC_GET,                      
  SD_BLE_GAP_RSSI_START,                         
  SD_BLE_GAP_RSSI_STOP,                          
  SD_BLE_GAP_SCAN_START,                        
  SD_BLE_GAP_SCAN_STOP,                         
  SD_BLE_GAP_CONNECT,                           
  SD_BLE_GAP_CONNECT_CANCEL,                    
  SD_BLE_GAP_RSSI_GET,                          
};



 
enum BLE_GAP_EVTS
{
  BLE_GAP_EVT_CONNECTED  = 0x10,     
  BLE_GAP_EVT_DISCONNECTED,                      
  BLE_GAP_EVT_CONN_PARAM_UPDATE,                 
  BLE_GAP_EVT_SEC_PARAMS_REQUEST,                
  BLE_GAP_EVT_SEC_INFO_REQUEST,                  
  BLE_GAP_EVT_PASSKEY_DISPLAY,                   
  BLE_GAP_EVT_AUTH_KEY_REQUEST,                  
  BLE_GAP_EVT_AUTH_STATUS,                       
  BLE_GAP_EVT_CONN_SEC_UPDATE,                   
  BLE_GAP_EVT_TIMEOUT,                           
  BLE_GAP_EVT_RSSI_CHANGED,                      
  BLE_GAP_EVT_ADV_REPORT,                        
  BLE_GAP_EVT_SEC_REQUEST,                       
  BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST,         
  BLE_GAP_EVT_SCAN_REQ_REPORT,                   
};



 
enum BLE_GAP_OPTS
{
  BLE_GAP_OPT_CH_MAP  = 0x20,        
  BLE_GAP_OPT_LOCAL_CONN_LATENCY,                
  BLE_GAP_OPT_PASSKEY,                           
  BLE_GAP_OPT_PRIVACY,                           
  BLE_GAP_OPT_SCAN_REQ_REPORT,                   
  BLE_GAP_OPT_COMPAT_MODE                        
};
 


 


 



 




 



 



 




 



 




 


 


 

 


 





 
#line 206 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"
 



 
#line 218 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"
 



 



  



 


  



 


  



 


  


 




 




 



 




 



 


 



 



 


 





 



 



 


 
#line 318 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"
 


 


 


 
#line 338 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gap.h"
 


 


 





 
 

 

 

 

 

 

 


 


 


 


 




 



 

 

 


 

 
typedef struct
{
  uint8_t addr_type;                     
  uint8_t addr[6];        
} ble_gap_addr_t;












 
typedef struct
{
  uint16_t min_conn_interval;          
  uint16_t max_conn_interval;          
  uint16_t slave_latency;              
  uint16_t conn_sup_timeout;           
} ble_gap_conn_params_t;










 
typedef struct
{
  uint8_t sm : 4;                      
  uint8_t lv : 4;                      

} ble_gap_conn_sec_mode_t;


 
typedef struct
{
  ble_gap_conn_sec_mode_t sec_mode;            
  uint8_t                 encr_key_size;       
} ble_gap_conn_sec_t;


 
typedef struct
{
  uint8_t irk[16];    
} ble_gap_irk_t;


 
typedef struct
{
  ble_gap_addr_t    **pp_addrs;         
  uint8_t             addr_count;       
  ble_gap_irk_t     **pp_irks;          
  uint8_t             irk_count;        
} ble_gap_whitelist_t;

  
typedef struct
{
  uint8_t ch_37_off : 1;   
  uint8_t ch_38_off : 1;   
  uint8_t ch_39_off : 1;   
} ble_gap_adv_ch_mask_t;

 
typedef struct
{
  uint8_t               type;                  
  ble_gap_addr_t       *p_peer_addr;           
  uint8_t               fp;                    
  ble_gap_whitelist_t  *p_whitelist;           
  uint16_t              interval;             

 
  uint16_t              timeout;               
  ble_gap_adv_ch_mask_t channel_mask;          
} ble_gap_adv_params_t;


 
typedef struct
{
  uint8_t                 active    : 1;         
  uint8_t                 selective : 1;         
  ble_gap_whitelist_t *   p_whitelist;           
  uint16_t                interval;              
  uint16_t                window;                
  uint16_t                timeout;               
} ble_gap_scan_params_t;


 
typedef struct
{
  uint8_t enc     : 1;                         
  uint8_t id      : 1;                         
  uint8_t sign    : 1;                         
} ble_gap_sec_kdist_t;


 
typedef struct
{
  uint8_t               bond    : 1;                
  uint8_t               mitm    : 1;                
  uint8_t               io_caps : 3;                
  uint8_t               oob     : 1;                
  uint8_t               min_key_size;               
  uint8_t               max_key_size;               
  ble_gap_sec_kdist_t   kdist_periph;               
  ble_gap_sec_kdist_t   kdist_central;              
} ble_gap_sec_params_t;


 
typedef struct
{
  uint8_t   ltk[16];    
  uint8_t   auth : 1;                    
  uint8_t   ltk_len : 7;                 
} ble_gap_enc_info_t;


 
typedef struct
{
  uint16_t  ediv;                        
  uint8_t   rand[8];  
} ble_gap_master_id_t;


 
typedef struct
{
  uint8_t   csrk[16];         
} ble_gap_sign_info_t;


 
typedef struct
{
  ble_gap_addr_t        peer_addr;               
  ble_gap_addr_t        own_addr;                
  uint8_t               irk_match :1;            
  uint8_t               irk_match_idx  :7;       
  ble_gap_conn_params_t conn_params;             
} ble_gap_evt_connected_t;


 
typedef struct
{
  uint8_t reason;                                
} ble_gap_evt_disconnected_t;


 
typedef struct
{
  ble_gap_conn_params_t conn_params;             
} ble_gap_evt_conn_param_update_t;


 
typedef struct
{
  ble_gap_sec_params_t peer_params;              
} ble_gap_evt_sec_params_request_t;


 
typedef struct
{
  ble_gap_addr_t      peer_addr;                      
  ble_gap_master_id_t master_id;                      
  uint8_t             enc_info  : 1;                  
  uint8_t             id_info   : 1;                  
  uint8_t             sign_info : 1;                  
} ble_gap_evt_sec_info_request_t;


 
typedef struct
{
  uint8_t passkey[6];          
} ble_gap_evt_passkey_display_t;


 
typedef struct
{
  uint8_t key_type;                              
} ble_gap_evt_auth_key_request_t;




 
typedef struct
{
  uint8_t lv1 : 1;                               
  uint8_t lv2 : 1;                               
  uint8_t lv3 : 1;                               
} ble_gap_sec_levels_t;


 
typedef struct
{
  ble_gap_enc_info_t    enc_info;              
  ble_gap_master_id_t   master_id;             
} ble_gap_enc_key_t;


 
typedef struct
{
  ble_gap_irk_t         id_info;               
  ble_gap_addr_t        id_addr_info;          
} ble_gap_id_key_t;


 
typedef struct
{
  ble_gap_enc_key_t     *p_enc_key;            
  ble_gap_id_key_t      *p_id_key;             
  ble_gap_sign_info_t   *p_sign_key;           
} ble_gap_sec_keys_t;




 
typedef struct
{
  ble_gap_sec_keys_t keys_periph;      
  ble_gap_sec_keys_t keys_central;     
} ble_gap_sec_keyset_t;


 
typedef struct
{
  uint8_t               auth_status;             
  uint8_t               error_src : 2;           
  uint8_t               bonded : 1;              
  ble_gap_sec_levels_t  sm1_levels;              
  ble_gap_sec_levels_t  sm2_levels;              
  ble_gap_sec_kdist_t   kdist_periph;            
  ble_gap_sec_kdist_t   kdist_central;           
} ble_gap_evt_auth_status_t;


 
typedef struct
{
  ble_gap_conn_sec_t conn_sec;                   
} ble_gap_evt_conn_sec_update_t;


 
typedef struct
{
  uint8_t src;                                   
} ble_gap_evt_timeout_t;


 
typedef struct
{
  int8_t  rssi;                                
} ble_gap_evt_rssi_changed_t;


 
typedef struct
{
  ble_gap_addr_t peer_addr;                      
  int8_t         rssi;                           
  uint8_t        scan_rsp : 1;                   
  uint8_t        type     : 2;                   
  uint8_t        dlen     : 5;                   
  uint8_t        data[31];     
} ble_gap_evt_adv_report_t;


 
typedef struct
{
  uint8_t    bond    : 1;                        
  uint8_t    mitm    : 1;                        
} ble_gap_evt_sec_request_t;


 
typedef struct
{
  ble_gap_conn_params_t conn_params;             
} ble_gap_evt_conn_param_update_request_t;


 
typedef struct
{
  int8_t                  rssi;               
  ble_gap_addr_t          peer_addr;          
} ble_gap_evt_scan_req_report_t;



 
typedef struct
{
  uint16_t conn_handle;                                      
  union                                                      
  {
    ble_gap_evt_connected_t                   connected;                     
    ble_gap_evt_disconnected_t                disconnected;                  
    ble_gap_evt_conn_param_update_t           conn_param_update;             
    ble_gap_evt_sec_params_request_t          sec_params_request;            
    ble_gap_evt_sec_info_request_t            sec_info_request;              
    ble_gap_evt_passkey_display_t             passkey_display;               
    ble_gap_evt_auth_key_request_t            auth_key_request;              
    ble_gap_evt_auth_status_t                 auth_status;                   
    ble_gap_evt_conn_sec_update_t             conn_sec_update;               
    ble_gap_evt_timeout_t                     timeout;                       
    ble_gap_evt_rssi_changed_t                rssi_changed;                  
    ble_gap_evt_adv_report_t                  adv_report;                    
    ble_gap_evt_sec_request_t                 sec_request;                   
    ble_gap_evt_conn_param_update_request_t   conn_param_update_request;     
    ble_gap_evt_scan_req_report_t             scan_req_report;               
  } params;                                                                  

} ble_gap_evt_t;






















 
typedef struct
{
  uint16_t conn_handle;                    
  uint8_t ch_map[5];                       
} ble_gap_opt_ch_map_t;























 
typedef struct
{
  uint16_t   conn_handle;                        
  uint16_t   requested_latency;                  
  uint16_t * p_actual_latency;                   
} ble_gap_opt_local_conn_latency_t;










 
typedef struct
{
  uint8_t * p_passkey;                           
} ble_gap_opt_passkey_t;
























 
typedef struct
{
  ble_gap_irk_t * p_irk;         
  uint16_t        interval_s;    
} ble_gap_opt_privacy_t;














 
typedef struct
{
   uint8_t enable : 1;                            
} ble_gap_opt_scan_req_report_t;











 
typedef struct
{
   uint8_t mode_1_enable : 1;                            
} ble_gap_opt_compat_mode_t;


 
typedef union
{
  ble_gap_opt_ch_map_t                  ch_map;                     
  ble_gap_opt_local_conn_latency_t      local_conn_latency;         
  ble_gap_opt_passkey_t                 passkey;                    
  ble_gap_opt_privacy_t                 privacy;                    
  ble_gap_opt_scan_req_report_t         scan_req_report;            
  ble_gap_opt_compat_mode_t             compat_mode;                
} ble_gap_opt_t;
 



 






































 
uint32_t __svc(SD_BLE_GAP_ADDRESS_SET) sd_ble_gap_address_set(uint8_t addr_cycle_mode, const ble_gap_addr_t *p_addr);








 
uint32_t __svc(SD_BLE_GAP_ADDRESS_GET) sd_ble_gap_address_get(ble_gap_addr_t *p_addr);

























 
uint32_t __svc(SD_BLE_GAP_ADV_DATA_SET) sd_ble_gap_adv_data_set(uint8_t const *p_data, uint8_t dlen, uint8_t const *p_sr_data, uint8_t srdlen);



















 
uint32_t __svc(SD_BLE_GAP_ADV_START) sd_ble_gap_adv_start(ble_gap_adv_params_t const *p_adv_params);






 
uint32_t __svc(SD_BLE_GAP_ADV_STOP) sd_ble_gap_adv_stop(void);























 
uint32_t __svc(SD_BLE_GAP_CONN_PARAM_UPDATE) sd_ble_gap_conn_param_update(uint16_t conn_handle, ble_gap_conn_params_t const *p_conn_params);














 
uint32_t __svc(SD_BLE_GAP_DISCONNECT) sd_ble_gap_disconnect(uint16_t conn_handle, uint8_t hci_status_code);










 
uint32_t __svc(SD_BLE_GAP_TX_POWER_SET) sd_ble_gap_tx_power_set(int8_t tx_power);








 
uint32_t __svc(SD_BLE_GAP_APPEARANCE_SET) sd_ble_gap_appearance_set(uint16_t appearance);








 
uint32_t __svc(SD_BLE_GAP_APPEARANCE_GET) sd_ble_gap_appearance_get(uint16_t *p_appearance);









 
uint32_t __svc(SD_BLE_GAP_PPCP_SET) sd_ble_gap_ppcp_set(ble_gap_conn_params_t const *p_conn_params);








 
uint32_t __svc(SD_BLE_GAP_PPCP_GET) sd_ble_gap_ppcp_get(ble_gap_conn_params_t *p_conn_params);












 
uint32_t __svc(SD_BLE_GAP_DEVICE_NAME_SET) sd_ble_gap_device_name_set(ble_gap_conn_sec_mode_t const *p_write_perm, uint8_t const *p_dev_name, uint16_t len);















 
uint32_t __svc(SD_BLE_GAP_DEVICE_NAME_GET) sd_ble_gap_device_name_get(uint8_t *p_dev_name, uint16_t *p_len);























 
uint32_t __svc(SD_BLE_GAP_AUTHENTICATE) sd_ble_gap_authenticate(uint16_t conn_handle, ble_gap_sec_params_t const *p_sec_params);
























 
uint32_t __svc(SD_BLE_GAP_SEC_PARAMS_REPLY) sd_ble_gap_sec_params_reply(uint16_t conn_handle, uint8_t sec_status, ble_gap_sec_params_t const *p_sec_params, ble_gap_sec_keyset_t const *p_sec_keyset);


















 
uint32_t __svc(SD_BLE_GAP_AUTH_KEY_REPLY) sd_ble_gap_auth_key_reply(uint16_t conn_handle, uint8_t key_type, uint8_t const *p_key);


















 
uint32_t __svc(SD_BLE_GAP_ENCRYPT) sd_ble_gap_encrypt(uint16_t conn_handle, ble_gap_master_id_t const *p_master_id, ble_gap_enc_info_t const *p_enc_info);

















 
uint32_t __svc(SD_BLE_GAP_SEC_INFO_REPLY) sd_ble_gap_sec_info_reply(uint16_t conn_handle, ble_gap_enc_info_t const *p_enc_info, ble_gap_irk_t const *p_id_info, ble_gap_sign_info_t const *p_sign_info);










 
uint32_t __svc(SD_BLE_GAP_CONN_SEC_GET) sd_ble_gap_conn_sec_get(uint16_t conn_handle, ble_gap_conn_sec_t *p_conn_sec);













 
uint32_t __svc(SD_BLE_GAP_RSSI_START) sd_ble_gap_rssi_start(uint16_t conn_handle, uint8_t threshold_dbm, uint8_t skip_count);












 
uint32_t __svc(SD_BLE_GAP_RSSI_STOP) sd_ble_gap_rssi_stop(uint16_t conn_handle);















 
uint32_t __svc(SD_BLE_GAP_RSSI_GET) sd_ble_gap_rssi_get(uint16_t conn_handle, int8_t *p_rssi);












 
uint32_t __svc(SD_BLE_GAP_SCAN_START) sd_ble_gap_scan_start(ble_gap_scan_params_t const *p_scan_params);






 
uint32_t __svc(SD_BLE_GAP_SCAN_STOP) sd_ble_gap_scan_stop(void);















 
uint32_t __svc(SD_BLE_GAP_CONNECT) sd_ble_gap_connect(ble_gap_addr_t const *p_peer_addr, ble_gap_scan_params_t const *p_scan_params, ble_gap_conn_params_t const *p_conn_params);






 
uint32_t __svc(SD_BLE_GAP_CONNECT_CANCEL) sd_ble_gap_connect_cancel(void);

 





 
#line 53 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_l2cap.h"


































 





 




#line 47 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_l2cap.h"
#line 48 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_l2cap.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_err.h"


































 















 



#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error.h"


































  
 




 

 




 




 

#line 73 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error.h"





 
#line 56 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_err.h"


 





 





 




 







 
#line 49 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_l2cap.h"
#line 50 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_l2cap.h"


 

 
enum BLE_L2CAP_SVCS 
{
  SD_BLE_L2CAP_CID_REGISTER = 0xB0,   
  SD_BLE_L2CAP_CID_UNREGISTER,                      
  SD_BLE_L2CAP_TX                                   
};

 
enum BLE_L2CAP_EVTS 
{
  BLE_L2CAP_EVT_RX  = 0x70           
};

 


 


 

 

 


 


 


 


 


 

 
typedef struct
{
  uint16_t   len;                                  
  uint16_t   cid;                                  
} ble_l2cap_header_t;


 
typedef struct
{
  ble_l2cap_header_t header;                       
  uint8_t    data[1];                              
} ble_l2cap_evt_rx_t;


 
typedef struct
{
  uint16_t conn_handle;                            
  union
  {
    ble_l2cap_evt_rx_t rx;                         
  } params;                                        
} ble_l2cap_evt_t;

 


 











 
uint32_t __svc(SD_BLE_L2CAP_CID_REGISTER) sd_ble_l2cap_cid_register(uint16_t cid);










 
uint32_t __svc(SD_BLE_L2CAP_CID_UNREGISTER) sd_ble_l2cap_cid_unregister(uint16_t cid);


















 
uint32_t __svc(SD_BLE_L2CAP_TX) sd_ble_l2cap_tx(uint16_t conn_handle, ble_l2cap_header_t const *p_header, uint8_t const *p_data);

 





 
#line 54 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatt.h"


































 





 




#line 47 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatt.h"
#line 48 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatt.h"



 

 


 



 



 

 


 
#line 76 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatt.h"
 


 


 


 



 


 
#line 124 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatt.h"
 




 
#line 158 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatt.h"
 



 


 

 


 

 
typedef struct
{
   
  uint8_t broadcast       :1;  
  uint8_t read            :1;  
  uint8_t write_wo_resp   :1;  
  uint8_t write           :1;  
  uint8_t notify          :1;  
  uint8_t indicate        :1;  
  uint8_t auth_signed_wr  :1;  
} ble_gatt_char_props_t;

 
typedef struct
{
   
  uint8_t reliable_wr     :1;  
  uint8_t wr_aux          :1;  
} ble_gatt_char_ext_props_t;



 




 
#line 55 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gattc.h"


































 





 




#line 47 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gattc.h"
#line 48 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gattc.h"
#line 49 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gattc.h"
#line 50 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gattc.h"


 

 
enum BLE_GATTC_SVCS
{
  SD_BLE_GATTC_PRIMARY_SERVICES_DISCOVER = 0x90,  
  SD_BLE_GATTC_RELATIONSHIPS_DISCOVER,                          
  SD_BLE_GATTC_CHARACTERISTICS_DISCOVER,                        
  SD_BLE_GATTC_DESCRIPTORS_DISCOVER,                            
  SD_BLE_GATTC_CHAR_VALUE_BY_UUID_READ,                         
  SD_BLE_GATTC_READ,                                            
  SD_BLE_GATTC_CHAR_VALUES_READ,                                
  SD_BLE_GATTC_WRITE,                                           
  SD_BLE_GATTC_HV_CONFIRM                                       
};



 
enum BLE_GATTC_EVTS
{
  BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP = 0x30,   
  BLE_GATTC_EVT_REL_DISC_RSP,                              
  BLE_GATTC_EVT_CHAR_DISC_RSP,                             
  BLE_GATTC_EVT_DESC_DISC_RSP,                             
  BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP,                 
  BLE_GATTC_EVT_READ_RSP,                                  
  BLE_GATTC_EVT_CHAR_VALS_READ_RSP,                        
  BLE_GATTC_EVT_WRITE_RSP,                                 
  BLE_GATTC_EVT_HVX,                                       
  BLE_GATTC_EVT_TIMEOUT                                    
};

 


 


 

 

 


 


 

 
typedef struct
{
  uint16_t          start_handle;  
  uint16_t          end_handle;    
} ble_gattc_handle_range_t;


 
typedef struct
{
  ble_uuid_t               uuid;           
  ble_gattc_handle_range_t handle_range;   
} ble_gattc_service_t;


 
typedef struct
{
  uint16_t            handle;            
  ble_gattc_service_t included_srvc;     
} ble_gattc_include_t;


 
typedef struct
{
  ble_uuid_t              uuid;                  
  ble_gatt_char_props_t   char_props;            
  uint8_t                 char_ext_props : 1;    
  uint16_t                handle_decl;           
  uint16_t                handle_value;          
} ble_gattc_char_t;


 
typedef struct
{
  uint16_t          handle;          
  ble_uuid_t        uuid;            
} ble_gattc_desc_t;


 
typedef struct
{
  uint8_t    write_op;                  
  uint8_t    flags;                     
  uint16_t   handle;                    
  uint16_t   offset;                    
  uint16_t   len;                       
  uint8_t   *p_value;                   
} ble_gattc_write_params_t;

 
typedef struct
{
  uint16_t             count;            
  ble_gattc_service_t services[1];       
} ble_gattc_evt_prim_srvc_disc_rsp_t;

 
typedef struct
{
  uint16_t             count;            
  ble_gattc_include_t includes[1];       
} ble_gattc_evt_rel_disc_rsp_t;

 
typedef struct
{
  uint16_t            count;           
  ble_gattc_char_t    chars[1];        
} ble_gattc_evt_char_disc_rsp_t;

 
typedef struct
{
  uint16_t            count;           
  ble_gattc_desc_t    descs[1];        
} ble_gattc_evt_desc_disc_rsp_t;

 
typedef struct 
{
  uint16_t            handle;           
  uint8_t             *p_value;        

 
} ble_gattc_handle_value_t;

 
typedef struct
{
  uint16_t                  count;             
  uint16_t                  value_len;         
  ble_gattc_handle_value_t  handle_value[1];   
} ble_gattc_evt_char_val_by_uuid_read_rsp_t;

 
typedef struct
{
  uint16_t            handle;          
  uint16_t            offset;          
  uint16_t            len;             
  uint8_t             data[1];         
} ble_gattc_evt_read_rsp_t;

 
typedef struct
{
  uint16_t            len;             
  uint8_t             values[1];       
} ble_gattc_evt_char_vals_read_rsp_t;

 
typedef struct
{
  uint16_t            handle;            
  uint8_t             write_op;          
  uint16_t            offset;            
  uint16_t            len;               
  uint8_t             data[1];           
} ble_gattc_evt_write_rsp_t;

 
typedef struct
{
  uint16_t            handle;          
  uint8_t             type;            
  uint16_t            len;             
  uint8_t             data[1];         
} ble_gattc_evt_hvx_t;

 
typedef struct
{
  uint8_t          src;                        
} ble_gattc_evt_timeout_t;

 
typedef struct
{
  uint16_t            conn_handle;                 
  uint16_t            gatt_status;                 
  uint16_t            error_handle;                
  union
  {
    ble_gattc_evt_prim_srvc_disc_rsp_t          prim_srvc_disc_rsp;          
    ble_gattc_evt_rel_disc_rsp_t                rel_disc_rsp;                
    ble_gattc_evt_char_disc_rsp_t               char_disc_rsp;               
    ble_gattc_evt_desc_disc_rsp_t               desc_disc_rsp;               
    ble_gattc_evt_char_val_by_uuid_read_rsp_t   char_val_by_uuid_read_rsp;   
    ble_gattc_evt_read_rsp_t                    read_rsp;                    
    ble_gattc_evt_char_vals_read_rsp_t          char_vals_read_rsp;          
    ble_gattc_evt_write_rsp_t                   write_rsp;                   
    ble_gattc_evt_hvx_t                         hvx;                         
    ble_gattc_evt_timeout_t                     timeout;                     
  } params;                                                                  
} ble_gattc_evt_t;
 


 


















 
uint32_t __svc(SD_BLE_GATTC_PRIMARY_SERVICES_DISCOVER) sd_ble_gattc_primary_services_discover(uint16_t conn_handle, uint16_t start_handle, ble_uuid_t const *p_srvc_uuid);
















 
uint32_t __svc(SD_BLE_GATTC_RELATIONSHIPS_DISCOVER) sd_ble_gattc_relationships_discover(uint16_t conn_handle, ble_gattc_handle_range_t const *p_handle_range);


















 
uint32_t __svc(SD_BLE_GATTC_CHARACTERISTICS_DISCOVER) sd_ble_gattc_characteristics_discover(uint16_t conn_handle, ble_gattc_handle_range_t const *p_handle_range);















 
uint32_t __svc(SD_BLE_GATTC_DESCRIPTORS_DISCOVER) sd_ble_gattc_descriptors_discover(uint16_t conn_handle, ble_gattc_handle_range_t const *p_handle_range);
















 
uint32_t __svc(SD_BLE_GATTC_CHAR_VALUE_BY_UUID_READ) sd_ble_gattc_char_value_by_uuid_read(uint16_t conn_handle, ble_uuid_t const *p_uuid, ble_gattc_handle_range_t const *p_handle_range);

















 
uint32_t __svc(SD_BLE_GATTC_READ) sd_ble_gattc_read(uint16_t conn_handle, uint16_t handle, uint16_t offset);















 
uint32_t __svc(SD_BLE_GATTC_CHAR_VALUES_READ) sd_ble_gattc_char_values_read(uint16_t conn_handle, uint16_t const *p_handles, uint16_t handle_count);






















 
uint32_t __svc(SD_BLE_GATTC_WRITE) sd_ble_gattc_write(uint16_t conn_handle, ble_gattc_write_params_t const *p_write_params);











 
uint32_t __svc(SD_BLE_GATTC_HV_CONFIRM) sd_ble_gattc_hv_confirm(uint16_t conn_handle, uint16_t handle);

 






 
#line 56 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"


































 





 




#line 47 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"
#line 48 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"
#line 49 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"
#line 50 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"
#line 51 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"
#line 52 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"


 



 
enum BLE_GATTS_SVCS
{
  SD_BLE_GATTS_SERVICE_ADD = 0xA0,  
  SD_BLE_GATTS_INCLUDE_ADD,                       
  SD_BLE_GATTS_CHARACTERISTIC_ADD,                
  SD_BLE_GATTS_DESCRIPTOR_ADD,                    
  SD_BLE_GATTS_VALUE_SET,                         
  SD_BLE_GATTS_VALUE_GET,                         
  SD_BLE_GATTS_HVX,                               
  SD_BLE_GATTS_SERVICE_CHANGED,                   
  SD_BLE_GATTS_RW_AUTHORIZE_REPLY,                 
  SD_BLE_GATTS_SYS_ATTR_SET,                        
  SD_BLE_GATTS_SYS_ATTR_GET,                      
};



 
enum BLE_GATTS_EVTS
{
  BLE_GATTS_EVT_WRITE = 0x50,        
  BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST,              
  BLE_GATTS_EVT_SYS_ATTR_MISSING,                  
  BLE_GATTS_EVT_HVC,                               
  BLE_GATTS_EVT_SC_CONFIRM,                        
  BLE_GATTS_EVT_TIMEOUT                            
};
 


 


 


 


 


 


 



 



 
#line 121 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"
 



 
#line 133 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble_gatts.h"
 


 




 


 



 


 


 



 


 

 


 



 
typedef struct
{
  uint8_t   service_changed:1;              
  uint32_t  attr_tab_size;                  
} ble_gatts_enable_params_t;

 
typedef struct
{
  ble_gap_conn_sec_mode_t read_perm;        
  ble_gap_conn_sec_mode_t write_perm;       
  uint8_t                 vlen       :1;    
  uint8_t                 vloc       :2;    
  uint8_t                 rd_auth    :1;     
  uint8_t                 wr_auth    :1;    
} ble_gatts_attr_md_t;


 
typedef struct
{
  ble_uuid_t          *p_uuid;           
  ble_gatts_attr_md_t *p_attr_md;        
  uint16_t             init_len;         
  uint16_t             init_offs;        
  uint16_t             max_len;          
  uint8_t*             p_value;         

 
} ble_gatts_attr_t;

 
typedef struct
{
  uint16_t  len;         
  uint16_t  offset;      
  uint8_t   *p_value;   

 
} ble_gatts_value_t;


 
typedef struct
{
  ble_uuid_t           srvc_uuid;        
  ble_uuid_t           char_uuid;        
  ble_uuid_t           desc_uuid;        
  uint16_t             srvc_handle;      
  uint16_t             value_handle;     
  uint8_t              type;             
} ble_gatts_attr_context_t;


 
typedef struct
{
  uint8_t          format;       
  int8_t           exponent;     
  uint16_t         unit;         
  uint8_t          name_space;   
  uint16_t         desc;         
} ble_gatts_char_pf_t;


 
typedef struct
{
  ble_gatt_char_props_t       char_props;                
  ble_gatt_char_ext_props_t   char_ext_props;            
  uint8_t                    *p_char_user_desc;          
  uint16_t                    char_user_desc_max_size;   
  uint16_t                    char_user_desc_size;        
  ble_gatts_char_pf_t*        p_char_pf;                 
  ble_gatts_attr_md_t*        p_user_desc_md;            
  ble_gatts_attr_md_t*        p_cccd_md;                 
  ble_gatts_attr_md_t*        p_sccd_md;                 
} ble_gatts_char_md_t;


 
typedef struct
{
  uint16_t          value_handle;        
  uint16_t          user_desc_handle;    
  uint16_t          cccd_handle;         
  uint16_t          sccd_handle;         
} ble_gatts_char_handles_t;


 
typedef struct
{
  uint16_t          handle;              
  uint8_t           type;                
  uint16_t          offset;              
  uint16_t         *p_len;               
  uint8_t          *p_data;              
} ble_gatts_hvx_params_t;

 
typedef struct
{
  uint16_t          gatt_status;         
  uint8_t           update : 1;          
  uint16_t          offset;              
  uint16_t          len;                 
  uint8_t          *p_data;              
} ble_gatts_read_authorize_params_t;

 
typedef struct
{
  uint16_t          gatt_status;         
} ble_gatts_write_authorize_params_t;

 
typedef struct
{
  uint8_t                               type;    
  union {
    ble_gatts_read_authorize_params_t   read;    
    ble_gatts_write_authorize_params_t  write;   
  } params;                                      
} ble_gatts_rw_authorize_reply_params_t;



 
typedef struct
{
  uint16_t                    handle;              
  uint8_t                     op;                  
  ble_gatts_attr_context_t    context;             
  uint16_t                    offset;              
  uint16_t                    len;                 
  uint8_t                     data[1];             
} ble_gatts_evt_write_t;

 
typedef struct
{
  uint16_t                    handle;              
  ble_gatts_attr_context_t    context;             
  uint16_t                    offset;              
} ble_gatts_evt_read_t;

 
typedef struct
{
  uint8_t                     type;              
  union {
    ble_gatts_evt_read_t      read;              
    ble_gatts_evt_write_t     write;             
  } request;                                     
} ble_gatts_evt_rw_authorize_request_t;

 
typedef struct
{
  uint8_t hint;                                  
} ble_gatts_evt_sys_attr_missing_t;


 
typedef struct
{
  uint16_t          handle;                        
} ble_gatts_evt_hvc_t;

 
typedef struct
{
  uint8_t          src;                        
} ble_gatts_evt_timeout_t;


 
typedef struct
{
  uint16_t conn_handle;                                        
  union
  {
    ble_gatts_evt_write_t                 write;               
    ble_gatts_evt_rw_authorize_request_t  authorize_request;   
    ble_gatts_evt_sys_attr_missing_t      sys_attr_missing;    
    ble_gatts_evt_hvc_t                   hvc;                 
    ble_gatts_evt_timeout_t               timeout;             
  } params;                                                    
} ble_gatts_evt_t;

 


 















 
uint32_t __svc(SD_BLE_GATTS_SERVICE_ADD) sd_ble_gatts_service_add(uint8_t type, ble_uuid_t const *p_uuid, uint16_t *p_handle);



















 
uint32_t __svc(SD_BLE_GATTS_INCLUDE_ADD) sd_ble_gatts_include_add(uint16_t service_handle, uint16_t inc_srvc_handle, uint16_t *p_include_handle);























 
uint32_t __svc(SD_BLE_GATTS_CHARACTERISTIC_ADD) sd_ble_gatts_characteristic_add(uint16_t service_handle, ble_gatts_char_md_t const *p_char_md, ble_gatts_attr_t const *p_attr_char_value, ble_gatts_char_handles_t *p_handles);

















 
uint32_t __svc(SD_BLE_GATTS_DESCRIPTOR_ADD) sd_ble_gatts_descriptor_add(uint16_t char_handle, ble_gatts_attr_t const *p_attr, uint16_t *p_handle);

















 
uint32_t __svc(SD_BLE_GATTS_VALUE_SET) sd_ble_gatts_value_set(uint16_t conn_handle, uint16_t handle, ble_gatts_value_t *p_value);





















 
uint32_t __svc(SD_BLE_GATTS_VALUE_GET) sd_ble_gatts_value_get(uint16_t conn_handle, uint16_t handle, ble_gatts_value_t *p_value);



































 
uint32_t __svc(SD_BLE_GATTS_HVX) sd_ble_gatts_hvx(uint16_t conn_handle, ble_gatts_hvx_params_t const *p_hvx_params);





















 
uint32_t __svc(SD_BLE_GATTS_SERVICE_CHANGED) sd_ble_gatts_service_changed(uint16_t conn_handle, uint16_t start_handle, uint16_t end_handle);















 
uint32_t __svc(SD_BLE_GATTS_RW_AUTHORIZE_REPLY) sd_ble_gatts_rw_authorize_reply(uint16_t conn_handle, ble_gatts_rw_authorize_reply_params_t const *p_rw_authorize_reply_params);



































  
uint32_t __svc(SD_BLE_GATTS_SYS_ATTR_SET) sd_ble_gatts_sys_attr_set(uint16_t conn_handle, uint8_t const *p_sys_attr_data, uint16_t len, uint32_t flags); 

 
























  
uint32_t __svc(SD_BLE_GATTS_SYS_ATTR_GET) sd_ble_gatts_sys_attr_get(uint16_t conn_handle, uint8_t *p_sys_attr_data, uint16_t *p_len, uint32_t flags); 

 





 
#line 57 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\ble.h"


 



 
enum BLE_COMMON_SVCS
{
  SD_BLE_ENABLE = 0x60,          
  SD_BLE_EVT_GET,                        
  SD_BLE_TX_BUFFER_COUNT_GET,            
  SD_BLE_UUID_VS_ADD,                    
  SD_BLE_UUID_DECODE,                    
  SD_BLE_UUID_ENCODE,                    
  SD_BLE_VERSION_GET,                    
  SD_BLE_USER_MEM_REPLY,                 
  SD_BLE_OPT_SET,                        
  SD_BLE_OPT_GET,                        
};



 
enum BLE_COMMON_EVTS
{
  BLE_EVT_TX_COMPLETE  = 0x01,   
  BLE_EVT_USER_MEM_REQUEST,              
  BLE_EVT_USER_MEM_RELEASE               
};



 
enum BLE_COMMON_OPTS
{
  BLE_COMMON_OPT_RADIO_CPU_MUTEX = 0x01     
};
 


 


 



 


 


 


 


 

 
typedef struct
{
  uint8_t          *p_mem;       
  uint16_t          len;         
} ble_user_mem_block_t;



 
typedef struct
{
  uint8_t count;                         
} ble_evt_tx_complete_t;

 
typedef struct
{
  uint8_t                     type;      
} ble_evt_user_mem_request_t;

 
typedef struct
{
  uint8_t                     type;        
  ble_user_mem_block_t        mem_block;   
} ble_evt_user_mem_release_t;


 
typedef struct
{
  uint16_t conn_handle;                  
  union
  {
    ble_evt_tx_complete_t           tx_complete;         
    ble_evt_user_mem_request_t      user_mem_request;    
    ble_evt_user_mem_release_t      user_mem_release;    
  } params;
} ble_common_evt_t;

 
typedef struct
{
  uint16_t evt_id;                       
  uint16_t evt_len;                      
} ble_evt_hdr_t;

 
typedef struct
{
  ble_evt_hdr_t header;                  
  union
  {
    ble_common_evt_t  common_evt;          
    ble_gap_evt_t     gap_evt;             
    ble_l2cap_evt_t   l2cap_evt;           
    ble_gattc_evt_t   gattc_evt;           
    ble_gatts_evt_t   gatts_evt;           
  } evt;
} ble_evt_t;




 
typedef struct
{
  uint8_t   version_number;              
  uint16_t  company_id;                  
  uint16_t  subversion_number;           
} ble_version_t;


















 
typedef struct
{
  uint8_t enable : 1;                           
} ble_common_opt_radio_cpu_mutex_t;

 
typedef union
{
  ble_common_opt_radio_cpu_mutex_t  radio_cpu_mutex;         
} ble_common_opt_t;

 
typedef union
{
  ble_common_opt_t  common_opt;          
  ble_gap_opt_t     gap_opt;             
} ble_opt_t;



 
typedef struct
{
  ble_gatts_enable_params_t  gatts_enable_params;  
} ble_enable_params_t;

 


 













 
uint32_t __svc(SD_BLE_ENABLE) sd_ble_enable(ble_enable_params_t * p_ble_enable_params);

























 
uint32_t __svc(SD_BLE_EVT_GET) sd_ble_evt_get(uint8_t *p_dest, uint16_t *p_len);



































 
uint32_t __svc(SD_BLE_TX_BUFFER_COUNT_GET) sd_ble_tx_buffer_count_get(uint8_t *p_count);



























 
uint32_t __svc(SD_BLE_UUID_VS_ADD) sd_ble_uuid_vs_add(ble_uuid128_t const *p_vs_uuid, uint8_t *p_uuid_type);



















                                                  
uint32_t __svc(SD_BLE_UUID_DECODE) sd_ble_uuid_decode(uint8_t uuid_le_len, uint8_t const *p_uuid_le, ble_uuid_t *p_uuid);













 
uint32_t __svc(SD_BLE_UUID_ENCODE) sd_ble_uuid_encode(ble_uuid_t const *p_uuid, uint8_t *p_uuid_le_len, uint8_t *p_uuid_le);











 
uint32_t __svc(SD_BLE_VERSION_GET) sd_ble_version_get(ble_version_t *p_version);













 
uint32_t __svc(SD_BLE_USER_MEM_REPLY) sd_ble_user_mem_reply(uint16_t conn_handle, ble_user_mem_block_t const *p_block);














 
uint32_t __svc(SD_BLE_OPT_SET) sd_ble_opt_set(uint32_t opt_id, ble_opt_t const *p_opt);

















 
uint32_t __svc(SD_BLE_OPT_GET) sd_ble_opt_get(uint32_t opt_id, ble_opt_t *p_opt);

 






 
#line 40 "..\\include\\nrf_adv_conn.h"

#line 42 "..\\include\\nrf_adv_conn.h"

void nrf_adv_conn_init(void);
void nrf_adv_conn_evt_handler(ble_evt_t *evt);

#line 37 "..\\nrf_adv_conn.c"
#line 1 "..\\include\\led_config.h"

































 




#line 1 "..\\..\\..\\SDK\\bsp\\boards.h"










 



#line 1 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"



#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"

 








































 





 



 









 

typedef enum {
 
  Reset_IRQn                    = -15,               
  NonMaskableInt_IRQn           = -14,               
  HardFault_IRQn                = -13,               
  SVCall_IRQn                   =  -5,               
  DebugMonitor_IRQn             =  -4,               
  PendSV_IRQn                   =  -2,               
  SysTick_IRQn                  =  -1,               
 
  POWER_CLOCK_IRQn              =   0,               
  RADIO_IRQn                    =   1,               
  UART0_IRQn                    =   2,               
  SPI0_TWI0_IRQn                =   3,               
  SPI1_TWI1_IRQn                =   4,               
  GPIOTE_IRQn                   =   6,               
  ADC_IRQn                      =   7,               
  TIMER0_IRQn                   =   8,               
  TIMER1_IRQn                   =   9,               
  TIMER2_IRQn                   =  10,               
  RTC0_IRQn                     =  11,               
  TEMP_IRQn                     =  12,               
  RNG_IRQn                      =  13,               
  ECB_IRQn                      =  14,               
  CCM_AAR_IRQn                  =  15,               
  WDT_IRQn                      =  16,               
  RTC1_IRQn                     =  17,               
  QDEC_IRQn                     =  18,               
  LPCOMP_IRQn                   =  19,               
  SWI0_IRQn                     =  20,               
  SWI1_IRQn                     =  21,               
  SWI2_IRQn                     =  22,               
  SWI3_IRQn                     =  23,               
  SWI4_IRQn                     =  24,               
  SWI5_IRQn                     =  25                
} IRQn_Type;




 


 
 
 

 




   

#line 1 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"
 







 

























 
























 




 


 

 













#line 100 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"


 







#line 125 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"

#line 127 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 



#line 292 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmInstr.h"



#line 685 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmInstr.h"

   

#line 128 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 271 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"


#line 307 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"


#line 632 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"

 


#line 129 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"








 
#line 154 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"

 






 
#line 170 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
  else {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








#line 120 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\system_nrf51.h"




























 








#line 39 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\system_nrf51.h"


extern uint32_t SystemCoreClock;     









 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 121 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


 
 
 




 


 

  #pragma push
  #pragma anon_unions
#line 148 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


typedef struct {
  volatile uint32_t  CPU0;                               
  volatile uint32_t  SPIS1;                              
  volatile uint32_t  RADIO;                              
  volatile uint32_t  ECB;                                
  volatile uint32_t  CCM;                                
  volatile uint32_t  AAR;                                
} AMLI_RAMPRI_Type;

typedef struct {
  volatile uint32_t  SCK;                                
  volatile uint32_t  MOSI;                               
  volatile uint32_t  MISO;                               
} SPIM_PSEL_Type;

typedef struct {
  volatile uint32_t  PTR;                                
  volatile uint32_t  MAXCNT;                             
  volatile const  uint32_t  AMOUNT;                             
} SPIM_RXD_Type;

typedef struct {
  volatile uint32_t  PTR;                                
  volatile uint32_t  MAXCNT;                             
  volatile const  uint32_t  AMOUNT;                             
} SPIM_TXD_Type;

typedef struct {
  volatile  uint32_t  EN;                                 
  volatile  uint32_t  DIS;                                
} PPI_TASKS_CHG_Type;

typedef struct {
  volatile uint32_t  EEP;                                
  volatile uint32_t  TEP;                                
} PPI_CH_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[30];
  volatile  uint32_t  TASKS_CONSTLAT;                     
  volatile  uint32_t  TASKS_LOWPWR;                       
  volatile const  uint32_t  RESERVED1[34];
  volatile uint32_t  EVENTS_POFWARN;                     
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile uint32_t  RESETREAS;                          
  volatile const  uint32_t  RESERVED4[9];
  volatile const  uint32_t  RAMSTATUS;                          
  volatile const  uint32_t  RESERVED5[53];
  volatile  uint32_t  SYSTEMOFF;                          
  volatile const  uint32_t  RESERVED6[3];
  volatile uint32_t  POFCON;                             
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  GPREGRET;                          
 
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  RAMON;                              
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RESET;                             
 
  volatile const  uint32_t  RESERVED10[3];
  volatile uint32_t  RAMONB;                             
  volatile const  uint32_t  RESERVED11[8];
  volatile uint32_t  DCDCEN;                             
  volatile const  uint32_t  RESERVED12[291];
  volatile uint32_t  DCDCFORCE;                          
} NRF_POWER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_HFCLKSTART;                   
  volatile  uint32_t  TASKS_HFCLKSTOP;                    
  volatile  uint32_t  TASKS_LFCLKSTART;                   
  volatile  uint32_t  TASKS_LFCLKSTOP;                    
  volatile  uint32_t  TASKS_CAL;                          
  volatile  uint32_t  TASKS_CTSTART;                      
  volatile  uint32_t  TASKS_CTSTOP;                       
  volatile const  uint32_t  RESERVED0[57];
  volatile uint32_t  EVENTS_HFCLKSTARTED;                
  volatile uint32_t  EVENTS_LFCLKSTARTED;                
  volatile const  uint32_t  RESERVED1;
  volatile uint32_t  EVENTS_DONE;                        
  volatile uint32_t  EVENTS_CTTO;                        
  volatile const  uint32_t  RESERVED2[124];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[63];
  volatile const  uint32_t  HFCLKRUN;                           
  volatile const  uint32_t  HFCLKSTAT;                          
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  LFCLKRUN;                           
  volatile const  uint32_t  LFCLKSTAT;                          
  volatile const  uint32_t  LFCLKSRCCOPY;                      
 
  volatile const  uint32_t  RESERVED5[62];
  volatile uint32_t  LFCLKSRC;                           
  volatile const  uint32_t  RESERVED6[7];
  volatile uint32_t  CTIV;                               
  volatile const  uint32_t  RESERVED7[5];
  volatile uint32_t  XTALFREQ;                           
} NRF_CLOCK_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[330];
  volatile uint32_t  PERR0;                              
  volatile uint32_t  RLENR0;                             
  volatile const  uint32_t  RESERVED1[52];
  volatile uint32_t  PROTENSET0;                         
  volatile uint32_t  PROTENSET1;                         
  volatile uint32_t  DISABLEINDEBUG;                     
  volatile uint32_t  PROTBLOCKSIZE;                      
} NRF_MPU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[448];
  volatile uint32_t  REPLACEADDR[8];                     
  volatile const  uint32_t  RESERVED1[24];
  volatile uint32_t  PATCHADDR[8];                       
  volatile const  uint32_t  RESERVED2[24];
  volatile uint32_t  PATCHEN;                            
  volatile uint32_t  PATCHENSET;                         
  volatile uint32_t  PATCHENCLR;                         
} NRF_PU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[896];
  AMLI_RAMPRI_Type RAMPRI;                           
} NRF_AMLI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_TXEN;                         
  volatile  uint32_t  TASKS_RXEN;                         
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_DISABLE;                      
  volatile  uint32_t  TASKS_RSSISTART;                    
  volatile  uint32_t  TASKS_RSSISTOP;                     
  volatile  uint32_t  TASKS_BCSTART;                      
  volatile  uint32_t  TASKS_BCSTOP;                       
  volatile const  uint32_t  RESERVED0[55];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_ADDRESS;                     
  volatile uint32_t  EVENTS_PAYLOAD;                     
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_DISABLED;                    
  volatile uint32_t  EVENTS_DEVMATCH;                    
  volatile uint32_t  EVENTS_DEVMISS;                     
  volatile uint32_t  EVENTS_RSSIEND;                    
 
  volatile const  uint32_t  RESERVED1[2];
  volatile uint32_t  EVENTS_BCMATCH;                     
  volatile const  uint32_t  RESERVED2[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[61];
  volatile const  uint32_t  CRCSTATUS;                          
  volatile const  uint32_t  CD;                                 
  volatile const  uint32_t  RXMATCH;                            
  volatile const  uint32_t  RXCRC;                              
  volatile const  uint32_t  DAI;                                
  volatile const  uint32_t  RESERVED5[60];
  volatile uint32_t  PACKETPTR;                          
  volatile uint32_t  FREQUENCY;                          
  volatile uint32_t  TXPOWER;                            
  volatile uint32_t  MODE;                               
  volatile uint32_t  PCNF0;                              
  volatile uint32_t  PCNF1;                              
  volatile uint32_t  BASE0;                              
  volatile uint32_t  BASE1;                              
  volatile uint32_t  PREFIX0;                            
  volatile uint32_t  PREFIX1;                            
  volatile uint32_t  TXADDRESS;                          
  volatile uint32_t  RXADDRESSES;                        
  volatile uint32_t  CRCCNF;                             
  volatile uint32_t  CRCPOLY;                            
  volatile uint32_t  CRCINIT;                            
  volatile uint32_t  TEST;                               
  volatile uint32_t  TIFS;                               
  volatile const  uint32_t  RSSISAMPLE;                         
  volatile const  uint32_t  RESERVED6;
  volatile const  uint32_t  STATE;                              
  volatile uint32_t  DATAWHITEIV;                        
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  BCC;                                
  volatile const  uint32_t  RESERVED8[39];
  volatile uint32_t  DAB[8];                             
  volatile uint32_t  DAP[8];                             
  volatile uint32_t  DACNF;                              
  volatile const  uint32_t  RESERVED9[56];
  volatile uint32_t  OVERRIDE0;                          
  volatile uint32_t  OVERRIDE1;                          
  volatile uint32_t  OVERRIDE2;                          
  volatile uint32_t  OVERRIDE3;                          
  volatile uint32_t  OVERRIDE4;                          
  volatile const  uint32_t  RESERVED10[561];
  volatile uint32_t  POWER;                              
} NRF_RADIO_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile  uint32_t  TASKS_STOPRX;                       
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile  uint32_t  TASKS_STOPTX;                       
  volatile const  uint32_t  RESERVED0[3];
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile const  uint32_t  RESERVED1[56];
  volatile uint32_t  EVENTS_CTS;                         
  volatile uint32_t  EVENTS_NCTS;                        
  volatile uint32_t  EVENTS_RXDRDY;                      
  volatile const  uint32_t  RESERVED2[4];
  volatile uint32_t  EVENTS_TXDRDY;                      
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED4[7];
  volatile uint32_t  EVENTS_RXTO;                        
  volatile const  uint32_t  RESERVED5[111];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED6[93];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED7[31];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  PSELRTS;                            
  volatile uint32_t  PSELTXD;                            
  volatile uint32_t  PSELCTS;                            
  volatile uint32_t  PSELRXD;                            
  volatile const  uint32_t  RXD;                               

 
  volatile  uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED9;
  volatile uint32_t  BAUDRATE;                           
  volatile const  uint32_t  RESERVED10[17];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED11[675];
  volatile uint32_t  POWER;                              
} NRF_UART_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[66];
  volatile uint32_t  EVENTS_READY;                       
  volatile const  uint32_t  RESERVED1[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[125];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELMISO;                           
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED7[681];
  volatile uint32_t  POWER;                              
} NRF_SPI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile const  uint32_t  RESERVED1[2];
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED2;
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile  uint32_t  TASKS_RESUME;                       
  volatile const  uint32_t  RESERVED3[56];
  volatile uint32_t  EVENTS_STOPPED;                     
  volatile uint32_t  EVENTS_RXDREADY;                    
  volatile const  uint32_t  RESERVED4[4];
  volatile uint32_t  EVENTS_TXDSENT;                     
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED6[4];
  volatile uint32_t  EVENTS_BB;                          
  volatile const  uint32_t  RESERVED7[3];
  volatile uint32_t  EVENTS_SUSPENDED;                   
  volatile const  uint32_t  RESERVED8[45];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED9[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED10[110];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED11[14];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  PSELSCL;                            
  volatile uint32_t  PSELSDA;                            
  volatile const  uint32_t  RESERVED13[2];
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED14;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED15[24];
  volatile uint32_t  ADDRESS;                            
  volatile const  uint32_t  RESERVED16[668];
  volatile uint32_t  POWER;                              
} NRF_TWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[9];
  volatile  uint32_t  TASKS_ACQUIRE;                      
  volatile  uint32_t  TASKS_RELEASE;                      
  volatile const  uint32_t  RESERVED1[54];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED2[8];
  volatile uint32_t  EVENTS_ACQUIRED;                    
  volatile const  uint32_t  RESERVED3[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED4[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED5[61];
  volatile const  uint32_t  SEMSTAT;                            
  volatile const  uint32_t  RESERVED6[15];
  volatile uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED7[47];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMISO;                           
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELCSN;                            
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RXDPTR;                             
  volatile uint32_t  MAXRX;                              
  volatile const  uint32_t  AMOUNTRX;                           
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  TXDPTR;                             
  volatile uint32_t  MAXTX;                              
  volatile const  uint32_t  AMOUNTTX;                           
  volatile const  uint32_t  RESERVED11;
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  DEF;                                
  volatile const  uint32_t  RESERVED13[24];
  volatile uint32_t  ORC;                                
  volatile const  uint32_t  RESERVED14[654];
  volatile uint32_t  POWER;                              
} NRF_SPIS_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[4];
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED1;
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile  uint32_t  TASKS_RESUME;                       
  volatile const  uint32_t  RESERVED2[56];
  volatile uint32_t  EVENTS_STOPPED;                     
  volatile const  uint32_t  RESERVED3[2];
  volatile uint32_t  EVENTS_ENDRX;                       
  volatile const  uint32_t  RESERVED4;
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  EVENTS_ENDTX;                       
  volatile const  uint32_t  RESERVED6[10];
  volatile uint32_t  EVENTS_STARTED;                     
  volatile const  uint32_t  RESERVED7[44];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED8[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED9[125];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED10;
  SPIM_PSEL_Type PSEL;                               
  volatile const  uint32_t  RESERVED11[4];
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED12[3];
  SPIM_RXD_Type RXD;                                 
  volatile const  uint32_t  RESERVED13;
  SPIM_TXD_Type TXD;                                 
  volatile const  uint32_t  RESERVED14;
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED15[26];
  volatile uint32_t  ORC;                                
  volatile const  uint32_t  RESERVED16[654];
  volatile uint32_t  POWER;                              
} NRF_SPIM_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_OUT[4];                       
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_IN[4];                       
  volatile const  uint32_t  RESERVED1[27];
  volatile uint32_t  EVENTS_PORT;                        
  volatile const  uint32_t  RESERVED2[97];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[129];
  volatile uint32_t  CONFIG[4];                          
  volatile const  uint32_t  RESERVED4[695];
  volatile uint32_t  POWER;                              
} NRF_GPIOTE_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  BUSY;                               
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_ADC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_COUNT;                        
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_SHUTDOWN;                     
  volatile const  uint32_t  RESERVED0[11];
  volatile  uint32_t  TASKS_CAPTURE[4];                   
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[44];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[126];
  volatile uint32_t  MODE;                               
  volatile uint32_t  BITMODE;                            
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED7[683];
  volatile uint32_t  POWER;                              
} NRF_TIMER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_TRIGOVRFLW;                   
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_TICK;                        
  volatile uint32_t  EVENTS_OVRFLW;                      
  volatile const  uint32_t  RESERVED1[14];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[109];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[13];
  volatile uint32_t  EVTEN;                              
  volatile uint32_t  EVTENSET;                          
 
  volatile uint32_t  EVTENCLR;                          
 
  volatile const  uint32_t  RESERVED4[110];
  volatile const  uint32_t  COUNTER;                            
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED5[13];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED6[683];
  volatile uint32_t  POWER;                              
} NRF_RTC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_DATARDY;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[127];
  volatile const  int32_t   TEMP;                               
  volatile const  uint32_t  RESERVED3[700];
  volatile uint32_t  POWER;                              
} NRF_TEMP_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_VALRDY;                      
  volatile const  uint32_t  RESERVED1[63];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[126];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  VALUE;                              
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_RNG_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTECB;                    

 
  volatile  uint32_t  TASKS_STOPECB;                     
 
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_ENDECB;                      
  volatile uint32_t  EVENTS_ERRORECB;                   
 
  volatile const  uint32_t  RESERVED1[127];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  ECBDATAPTR;                         
  volatile const  uint32_t  RESERVED3[701];
  volatile uint32_t  POWER;                              
} NRF_ECB_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                       
 
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_RESOLVED;                    
  volatile uint32_t  EVENTS_NOTRESOLVED;                 
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  NIRK;                               
  volatile uint32_t  IRKPTR;                             
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  ADDRPTR;                            
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED6[697];
  volatile uint32_t  POWER;                              
} NRF_AAR_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_KSGEN;                       
 
  volatile  uint32_t  TASKS_CRYPT;                       
 
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_ENDKSGEN;                    
  volatile uint32_t  EVENTS_ENDCRYPT;                    
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  MICSTATUS;                          
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  MODE;                               
  volatile uint32_t  CNFPTR;                             
  volatile uint32_t  INPTR;                              
  volatile uint32_t  OUTPTR;                             
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED5[697];
  volatile uint32_t  POWER;                              
} NRF_CCM_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile const  uint32_t  RESERVED0[63];
  volatile uint32_t  EVENTS_TIMEOUT;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  RUNSTATUS;                          
  volatile const  uint32_t  REQSTATUS;                          
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  CRV;                                
  volatile uint32_t  RREN;                               
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED4[60];
  volatile  uint32_t  RR[8];                              
  volatile const  uint32_t  RESERVED5[631];
  volatile uint32_t  POWER;                              
} NRF_WDT_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_READCLRACC;                  
 
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_SAMPLERDY;                   
  volatile uint32_t  EVENTS_REPORTRDY;                  
 
  volatile uint32_t  EVENTS_ACCOF;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[125];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  LEDPOL;                             
  volatile uint32_t  SAMPLEPER;                          
  volatile const  int32_t   SAMPLE;                             
  volatile uint32_t  REPORTPER;                          
  volatile const  int32_t   ACC;                                
  volatile const  int32_t   ACCREAD;                           
 
  volatile uint32_t  PSELLED;                            
  volatile uint32_t  PSELA;                              
  volatile uint32_t  PSELB;                              
  volatile uint32_t  DBFEN;                              
  volatile const  uint32_t  RESERVED4[5];
  volatile uint32_t  LEDPRE;                             
  volatile const  uint32_t  ACCDBL;                             
  volatile const  uint32_t  ACCDBLREAD;                        
 
  volatile const  uint32_t  RESERVED5[684];
  volatile uint32_t  POWER;                              
} NRF_QDEC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_SAMPLE;                       
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_DOWN;                        
  volatile uint32_t  EVENTS_UP;                          
  volatile uint32_t  EVENTS_CROSS;                       
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  PSEL;                               
  volatile uint32_t  REFSEL;                             
  volatile uint32_t  EXTREFSEL;                          
  volatile const  uint32_t  RESERVED5[4];
  volatile uint32_t  ANADETECT;                          
  volatile const  uint32_t  RESERVED6[694];
  volatile uint32_t  POWER;                              
} NRF_LPCOMP_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  UNUSED;                             
} NRF_SWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[256];
  volatile const  uint32_t  READY;                              
  volatile const  uint32_t  RESERVED1[64];
  volatile uint32_t  CONFIG;                             
  
  union {
    volatile uint32_t  ERASEPCR1;                        
    volatile uint32_t  ERASEPAGE;                        
  };
  volatile uint32_t  ERASEALL;                           
  volatile uint32_t  ERASEPCR0;                          
  volatile uint32_t  ERASEUICR;                          
} NRF_NVMC_Type;


 
 
 




 

typedef struct {                                     
  PPI_TASKS_CHG_Type TASKS_CHG[4];                   
  volatile const  uint32_t  RESERVED0[312];
  volatile uint32_t  CHEN;                               
  volatile uint32_t  CHENSET;                            
  volatile uint32_t  CHENCLR;                            
  volatile const  uint32_t  RESERVED1;
  PPI_CH_Type CH[16];                                
  volatile const  uint32_t  RESERVED2[156];
  volatile uint32_t  CHG[4];                             
} NRF_PPI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[4];
  volatile const  uint32_t  CODEPAGESIZE;                       
  volatile const  uint32_t  CODESIZE;                           
  volatile const  uint32_t  RESERVED1[4];
  volatile const  uint32_t  CLENR0;                             
  volatile const  uint32_t  PPFC;                               
  volatile const  uint32_t  RESERVED2;
  volatile const  uint32_t  NUMRAMBLOCK;                        
  
  union {
    volatile const  uint32_t  SIZERAMBLOCK[4];                 

 
    volatile const  uint32_t  SIZERAMBLOCKS;                    
  };
  volatile const  uint32_t  RESERVED3[5];
  volatile const  uint32_t  CONFIGID;                           
  volatile const  uint32_t  DEVICEID[2];                        
  volatile const  uint32_t  RESERVED4[6];
  volatile const  uint32_t  ER[4];                              
  volatile const  uint32_t  IR[4];                              
  volatile const  uint32_t  DEVICEADDRTYPE;                     
  volatile const  uint32_t  DEVICEADDR[2];                      
  volatile const  uint32_t  OVERRIDEEN;                         
  volatile const  uint32_t  NRF_1MBIT[5];                      
 
  volatile const  uint32_t  RESERVED5[10];
  volatile const  uint32_t  BLE_1MBIT[5];                      
 
} NRF_FICR_Type;


 
 
 




 

typedef struct {                                     
  volatile uint32_t  CLENR0;                             
  volatile uint32_t  RBPCONF;                            
  volatile uint32_t  XTALFREQ;                           
  volatile const  uint32_t  RESERVED0;
  volatile const  uint32_t  FWID;                               
  
  union {
    volatile uint32_t  NRFFW[15];                        
    volatile uint32_t  BOOTLOADERADDR;                   
  };
  volatile uint32_t  NRFHW[12];                          
  volatile uint32_t  CUSTOMER[32];                       
} NRF_UICR_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[321];
  volatile uint32_t  OUT;                                
  volatile uint32_t  OUTSET;                             
  volatile uint32_t  OUTCLR;                             
  volatile const  uint32_t  IN;                                 
  volatile uint32_t  DIR;                                
  volatile uint32_t  DIRSET;                             
  volatile uint32_t  DIRCLR;                             
  volatile const  uint32_t  RESERVED1[120];
  volatile uint32_t  PIN_CNF[32];                        
} NRF_GPIO_Type;


 

  #pragma pop
#line 1214 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"




 
 
 

#line 1256 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


 
 
 

#line 1296 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


   
   
   








#line 5 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"




























 



 

 
 

 
 

 






 






 






 
 

 






 






 






 
 

 



 
 

 





 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 





 
 

 






 
#line 182 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 190 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 199 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 






 
 

 



 
 

 






 
 

 
 

 
#line 241 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 253 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 265 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 277 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 289 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 301 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 313 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 325 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 340 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 352 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 364 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 376 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 388 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 400 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 412 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 424 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 439 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 451 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 463 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 475 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 487 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 499 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 511 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 523 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 538 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 550 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 562 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 574 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 586 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 598 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 610 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 622 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 637 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 649 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 661 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 673 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 685 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 697 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 709 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 721 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 736 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 748 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 760 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 772 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 784 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 796 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 808 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 820 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"


 
 

 
 

 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 





 
 

 






 
 

 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 





 
 

 





 
 

 





 






 
 

 






 
 

 






 
 

 



 
 

 






 
 

 
 

 






 






 
 

 






 






 
 

 






 
 

 
 

 





 
 

 



 



 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 
#line 2682 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 






 





 






 
 

 
 

 






 






 






 






 






 
 

 






 






 






 






 






 
 

 





 
#line 2798 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 



 






 
 

 






 
 

 
 

 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 
 

 
#line 2950 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 2965 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 
 

 





 
 

 
 

 





 
 

 






 
 

 





 
 

 






 
 

 
 

 






 
 

 






 
 

 





 





 





 





 





 





 





 
 

 





 





 





 





 
 

 




 
 

 
#line 3753 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 





 
 

 



 
 

 





 





 





 





 
 

 





 
 

 





 





 





 





 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 
 

 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 







 
 

 
 

 





 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 
#line 4877 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 
#line 4899 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 
#line 5184 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 5195 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 



 



 
 

 





 





 



 



 



 
 

 



 



 



 



 
 

 



 



 



 



 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 





 
#line 5350 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 





 





 
 

 



 
 

 



 
 

 
#line 5409 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 



 



 



 



 



 



 



 



 





 





 





 





 





 





 





 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 



 
 

 






 
 

 
 

 





 
 

 






 
 

 






 
 

 





 
 

 



 
 

 






 
 

 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 





 





 





 





 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 



 
 

 



 
 

 
#line 5928 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 





 





 
 

 






 
 

 
 

 





 
 

 






 






 






 






 






 
 

 






 






 






 






 






 
 

 





 
 

 
#line 6070 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 





 





 
 

 



 
 

 






 
 

 
 

 





 
 

 






 






 
 

 






 






 
 

 
#line 6207 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 





 





 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 
#line 6475 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 






 
 

 





 
 

 



 
 

 



 
 

 






 
 

 



 
 

 






 
 

 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 
#line 6844 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 





 
 

 






 
 

 
 

 





 





 
 

 





 
 

 




 
 

 
 

 






 
 

 






 
 

 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 
 

 




 
 

 






 
#line 6 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"



#line 7 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"












 






 
typedef enum
{
    NRF_GPIO_PORT_DIR_OUTPUT,       
    NRF_GPIO_PORT_DIR_INPUT         
} nrf_gpio_port_dir_t;




 
typedef enum
{
    NRF_GPIO_PIN_DIR_INPUT,   
    NRF_GPIO_PIN_DIR_OUTPUT   
} nrf_gpio_pin_dir_t;




 
typedef enum
{
    NRF_GPIO_PORT_SELECT_PORT0 = 0,           
    NRF_GPIO_PORT_SELECT_PORT1,               
    NRF_GPIO_PORT_SELECT_PORT2,               
    NRF_GPIO_PORT_SELECT_PORT3,               
} nrf_gpio_port_select_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOPULL   = (0x00UL),                 
    NRF_GPIO_PIN_PULLDOWN = (0x01UL),                 
    NRF_GPIO_PIN_PULLUP   = (0x03UL),                   
} nrf_gpio_pin_pull_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOSENSE    = (0x00UL),              
    NRF_GPIO_PIN_SENSE_LOW  = (0x03UL),                   
    NRF_GPIO_PIN_SENSE_HIGH = (0x02UL),                  
} nrf_gpio_pin_sense_t;











 
static __inline void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end);













 
static __inline void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config);








 
static __inline void nrf_gpio_cfg_output(uint32_t pin_number);









 
static __inline void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config);





 
static __inline void nrf_gpio_cfg_default(uint32_t pin_number);






 
static __inline void nrf_gpio_cfg_watcher(uint32_t pin_number);






 
static __inline void nrf_gpio_input_disconnect(uint32_t pin_number);









 
static __inline void nrf_gpio_cfg_sense_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config, nrf_gpio_pin_sense_t sense_config);







 
static __inline void nrf_gpio_cfg_sense_set(uint32_t pin_number, nrf_gpio_pin_sense_t sense_config);








 
static __inline void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction);









 
static __inline void nrf_gpio_pin_set(uint32_t pin_number);









 
static __inline void nrf_gpio_pin_clear(uint32_t pin_number);









 
static __inline void nrf_gpio_pin_toggle(uint32_t pin_number);













 
static __inline void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value);














 
static __inline uint32_t nrf_gpio_pin_read(uint32_t pin_number);








 
static __inline uint32_t nrf_gpio_pins_read(void);








 
static __inline nrf_gpio_pin_sense_t nrf_gpio_pin_sense_get(uint32_t pin_number);














 
static __inline void nrf_gpio_word_byte_write(volatile uint32_t * word_address, uint8_t byte_no, uint8_t value);













 
static __inline uint8_t nrf_gpio_word_byte_read(const volatile uint32_t* word_address, uint8_t byte_no);







 
static __inline void nrf_gpio_port_dir_set(nrf_gpio_port_select_t port, nrf_gpio_port_dir_t dir);







 
static __inline uint8_t nrf_gpio_port_read(nrf_gpio_port_select_t port);









 
static __inline void nrf_gpio_port_write(nrf_gpio_port_select_t port, uint8_t value);











 
static __inline void nrf_gpio_port_set(nrf_gpio_port_select_t port, uint8_t set_mask);











 
static __inline void nrf_gpio_port_clear(nrf_gpio_port_select_t port, uint8_t clr_mask);


static __inline void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | ((0x00UL) << (2UL))
                                        | ((1UL) << (1UL))
                                        | ((1UL) << (0UL));
    }
}

static __inline void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
    }
}

static __inline void nrf_gpio_cfg_output(uint32_t pin_number)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                            | ((0x00UL) << (8UL))
                                            | ((0x00UL) << (2UL))
                                            | ((1UL) << (1UL))
                                            | ((1UL) << (0UL));
}

static __inline void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}

static __inline void nrf_gpio_cfg_default(uint32_t pin_number)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (NRF_GPIO_PIN_NOPULL << (2UL))
                                        | ((1UL) << (1UL))
                                        | ((0UL) << (0UL));
}

static __inline void nrf_gpio_cfg_watcher(uint32_t pin_number)
{
     
    uint32_t cnf = ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] & ~(0x1UL << (1UL));
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = cnf | ((0UL) << (1UL));
}

static __inline void nrf_gpio_input_disconnect(uint32_t pin_number)
{
     
    uint32_t cnf = ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] & ~(0x1UL << (1UL));
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = cnf | ((1UL) << (1UL));
}

static __inline void nrf_gpio_cfg_sense_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config, nrf_gpio_pin_sense_t sense_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = (sense_config << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}

static __inline void nrf_gpio_cfg_sense_set(uint32_t pin_number, nrf_gpio_pin_sense_t sense_config)
{
     
    
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] &= ~(0x3UL << (16UL));
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] |= (sense_config << (16UL));
}

static __inline void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction)
{
    if(direction == NRF_GPIO_PIN_DIR_INPUT)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] =
          ((0x00UL) << (16UL))
        | ((0x00UL) << (8UL))
        | ((0x00UL) << (2UL))
        | ((0UL) << (1UL))
        | ((0UL) << (0UL));
    }
    else
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->DIRSET = (1UL << pin_number);
    }
}

static __inline void nrf_gpio_pin_set(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTSET = (1UL << pin_number);
}

static __inline void nrf_gpio_pin_clear(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR = (1UL << pin_number);
}

static __inline void nrf_gpio_pin_toggle(uint32_t pin_number)
{
    const uint32_t pin_bit   = 1UL << pin_number;
    const uint32_t pin_state = ((((NRF_GPIO_Type *) 0x50000000UL)->OUT >> pin_number) & 1UL);

    if (pin_state == 0)
    {
        
        ((NRF_GPIO_Type *) 0x50000000UL)->OUTSET = pin_bit;
    }
    else
    {
        
        ((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR = pin_bit;
    }
}

static __inline void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value)
{
    if (value == 0)
    {
        nrf_gpio_pin_clear(pin_number);
    }
    else
    {
        nrf_gpio_pin_set(pin_number);
    }
}

static __inline uint32_t nrf_gpio_pin_read(uint32_t pin_number)
{
    return  ((((NRF_GPIO_Type *) 0x50000000UL)->IN >> pin_number) & 1UL);
}

static __inline uint32_t nrf_gpio_pins_read(void)
{
    return ((NRF_GPIO_Type *) 0x50000000UL)->IN;
}

static __inline nrf_gpio_pin_sense_t nrf_gpio_pin_sense_get(uint32_t pin_number)
{
    return (nrf_gpio_pin_sense_t)((((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] & (0x3UL << (16UL))) >> (16UL));
}

static __inline void nrf_gpio_word_byte_write(volatile uint32_t * word_address, uint8_t byte_no, uint8_t value)
{
    *((volatile uint8_t*)(word_address) + byte_no) = value;
}

static __inline uint8_t nrf_gpio_word_byte_read(const volatile uint32_t* word_address, uint8_t byte_no)
{
    return (*((const volatile uint8_t*)(word_address) + byte_no));
}

static __inline void nrf_gpio_port_dir_set(nrf_gpio_port_select_t port, nrf_gpio_port_dir_t dir)
{
    if (dir == NRF_GPIO_PORT_DIR_OUTPUT)
    {
        nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->DIRSET, port, 0xFF);
    }
    else
    {
        nrf_gpio_range_cfg_input(port*8, (port+1)*8-1, NRF_GPIO_PIN_NOPULL);
    }
}

static __inline uint8_t nrf_gpio_port_read(nrf_gpio_port_select_t port)
{
    return nrf_gpio_word_byte_read(&((NRF_GPIO_Type *) 0x50000000UL)->IN, port);
}

static __inline void nrf_gpio_port_write(nrf_gpio_port_select_t port, uint8_t value)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUT, port, value);
}

static __inline void nrf_gpio_port_set(nrf_gpio_port_select_t port, uint8_t set_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTSET, port, set_mask);
}

static __inline void nrf_gpio_port_clear(nrf_gpio_port_select_t port, uint8_t clr_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR, port, clr_mask);
}

 

#line 16 "..\\..\\..\\SDK\\bsp\\boards.h"

#line 1 "..\\..\\..\\SDK\\bsp\\pca_RHC.h"










 

























 











































































#line 39 "..\\..\\..\\SDK\\bsp\\boards.h"





















#line 40 "..\\include\\led_config.h"
#line 41 "..\\include\\led_config.h"

 






void led_config(uint8_t led, uint8_t conf);


#line 38 "..\\nrf_adv_conn.c"

#line 1 "..\\..\\..\\rbc_mesh\\rbc_mesh.h"

































 

#line 39 "..\\..\\..\\rbc_mesh\\rbc_mesh.h"
#line 40 "..\\..\\..\\rbc_mesh\\rbc_mesh.h"
#line 41 "..\\..\\..\\rbc_mesh\\rbc_mesh.h"
#line 42 "..\\..\\..\\rbc_mesh\\rbc_mesh.h"










 
typedef uint16_t rbc_mesh_value_handle_t;
    
 
typedef enum
{
    RBC_MESH_EVENT_TYPE_UPDATE_VAL,       
    RBC_MESH_EVENT_TYPE_CONFLICTING_VAL,  
    RBC_MESH_EVENT_TYPE_NEW_VAL,          
    RBC_MESH_EVENT_TYPE_INITIALIZED       
} rbc_mesh_event_type_t;




 
typedef struct
{
    rbc_mesh_event_type_t event_type;         
    rbc_mesh_value_handle_t value_handle;     
    uint8_t* data;                       
    uint8_t data_len;                    
    ble_gap_addr_t originator_address;  
} rbc_mesh_event_t;








 
typedef enum
{
    RBC_MESH_RADIO_MODE_1MBIT,
    RBC_MESH_RADIO_MODE_2MBIT,
    RBC_MESH_RADIO_MODE_250KBIT,
    RBC_MESH_RADIO_MODE_BLE_1MBIT
} rbc_mesh_radio_mode_t;








 
typedef enum
{
    RBC_MESH_PACKET_FORMAT_ORIGINAL,
    RBC_MESH_PACKET_FORMAT_ADV_COMPATIBLE
} rbc_mesh_packet_format_t;





























 
typedef struct
{
    uint32_t access_addr;
    uint8_t channel;
    uint8_t handle_count;
    uint32_t adv_int_ms;
    rbc_mesh_radio_mode_t radio_mode;
    rbc_mesh_packet_format_t packet_format;
} rbc_mesh_init_params_t;



 













 
uint32_t rbc_mesh_init(rbc_mesh_init_params_t init_params);
















 
uint32_t rbc_mesh_value_set(uint8_t handle, uint8_t* data, uint16_t len);
















 
uint32_t rbc_mesh_value_enable(uint8_t handle);















 
uint32_t rbc_mesh_value_disable(uint8_t handle);
















 
uint32_t rbc_mesh_value_get(uint8_t handle, 
    uint8_t* data, 
    uint16_t* len, 
    ble_gap_addr_t* origin_addr);








 
uint32_t rbc_mesh_access_address_get(uint32_t* access_address);








 
uint32_t rbc_mesh_channel_get(uint8_t* ch);








 
uint32_t rbc_mesh_handle_count_get(uint8_t* handle_count);





















 
uint32_t rbc_mesh_value_get(uint8_t handle, 
    uint8_t* data, 
    uint16_t* len, 
    ble_gap_addr_t* origin_addr);








 
uint32_t rbc_mesh_access_address_get(uint32_t* access_address);








 
uint32_t rbc_mesh_channel_get(uint8_t* ch);








 
uint32_t rbc_mesh_handle_count_get(uint8_t* handle_count);








 
uint32_t rbc_mesh_adv_int_get(uint32_t* adv_int_ms);






















 
uint32_t rbc_mesh_ble_evt_handler(ble_evt_t* evt);







 
uint32_t rbc_mesh_sd_irq_handler(void);










 
void rbc_mesh_event_handler(rbc_mesh_event_t* evt);



#line 40 "..\\nrf_adv_conn.c"

#line 42 "..\\nrf_adv_conn.c"
#line 1 "..\\..\\..\\..\\..\\..\\components\\ble\\common\\ble_advdata.h"










 








 




#line 26 "..\\..\\..\\..\\..\\..\\components\\ble\\common\\ble_advdata.h"
#line 27 "..\\..\\..\\..\\..\\..\\components\\ble\\common\\ble_advdata.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 185 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 201 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 224 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 239 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 262 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 494 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"



 

#line 28 "..\\..\\..\\..\\..\\..\\components\\ble\\common\\ble_advdata.h"
#line 29 "..\\..\\..\\..\\..\\..\\components\\ble\\common\\ble_advdata.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"










 








 




#line 26 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"
#line 27 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\compiler_abstraction.h"




























 



 


    



    



    



    



    

  
#line 123 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\compiler_abstraction.h"

 

#line 28 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"

enum
{
    UNIT_0_625_MS = 625,                                 
    UNIT_1_25_MS  = 1250,                                
    UNIT_10_MS    = 10000                                
};














 

#line 58 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"


 
typedef uint8_t uint16_le_t[2];

 
typedef uint8_t uint32_le_t[4];

 
typedef struct
{
    uint16_t  size;                  
    uint8_t * p_data;                
} uint8_array_t;
    






 








 





 












 











 
static __inline uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}
    






 
static __inline uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}






 
static __inline uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) | 
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}






 
static __inline uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}
    



















 
static __inline uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}






 
static __inline _Bool is_word_aligned(void * p)
{
    return (((uintptr_t)p & 0x03) == 0);
}



 
#line 30 "..\\..\\..\\..\\..\\..\\components\\ble\\common\\ble_advdata.h"


 
typedef enum
{
    BLE_ADVDATA_NO_NAME,                                               
    BLE_ADVDATA_SHORT_NAME,                                            
    BLE_ADVDATA_FULL_NAME                                              
} ble_advdata_name_type_t;

 
typedef struct
{
    uint16_t                     uuid_cnt;                             
    ble_uuid_t *                 p_uuids;                              
} ble_advdata_uuid_list_t;

 
typedef struct
{
    uint16_t                     min_conn_interval;                    
    uint16_t                     max_conn_interval;                    
} ble_advdata_conn_int_t;

 
typedef struct
{
    uint16_t                     company_identifier;                   
    uint8_array_t                data;                                 
} ble_advdata_manuf_data_t;

 
typedef struct
{
    uint16_t                     service_uuid;                         
    uint8_array_t                data;                                 
} ble_advdata_service_data_t;


 
typedef struct
{
    ble_advdata_name_type_t      name_type;                            
    uint8_t                      short_name_len;                       
    _Bool                         include_appearance;                   
    uint8_t                      flags;                                
    int8_t *                     p_tx_power_level;                     
    ble_advdata_uuid_list_t      uuids_more_available;                 
    ble_advdata_uuid_list_t      uuids_complete;                       
    ble_advdata_uuid_list_t      uuids_solicited;                      
    ble_advdata_conn_int_t *     p_slave_conn_int;                     
    ble_advdata_manuf_data_t *   p_manuf_specific_data;                
    ble_advdata_service_data_t * p_service_data_array;                 
    uint8_t                      service_data_count;                   
} ble_advdata_t;























 
uint32_t ble_advdata_set(const ble_advdata_t * p_advdata, const ble_advdata_t * p_srdata);



 
#line 43 "..\\nrf_adv_conn.c"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\nrf_assert.h"







 



 




#line 18 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\nrf_assert.h"
#line 19 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\nrf_assert.h"

#line 56 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\nrf_assert.h"
__weak void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name);


#line 44 "..\\nrf_adv_conn.c"
#line 45 "..\\nrf_adv_conn.c"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"










 
 







 




#line 26 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"
#line 27 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"
#line 28 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"






 
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);




 
#line 54 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"



     
#line 67 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"
    



 
#line 81 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"



 
#line 46 "..\\nrf_adv_conn.c"

#line 48 "..\\nrf_adv_conn.c"
#line 49 "..\\nrf_adv_conn.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 138 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 957 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 50 "..\\nrf_adv_conn.c"
                            





 
      
 


 
static ble_gap_adv_params_t ble_adv_params = {
    0x00,   
    0,                       
    0x00,         
    0,                       
    (320),        
    0,                           
		{
			0, 0, 0
		}
};

static ble_advdata_t ble_adv_data;
static ble_gap_sec_params_t ble_gap_bond_params = {

    .bond = 0,                       
    .mitm = 0,                       
    .io_caps = 0x03,    
    .oob = 0,                       
    .min_key_size = 7,                       
    .max_key_size = 16                       
};



 
 
static void ble_gatts_event_handler(ble_evt_t* evt)
{
    switch (evt->header.evt_id)
    {
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        sd_ble_gatts_sys_attr_set(evt->evt.gatts_evt.conn_handle, 0, 0, 0);
        break;

    case BLE_GATTS_EVT_WRITE:
        break;

    default:
        break;
    }
}

static void ble_gap_event_handler(ble_evt_t* evt)
{
    switch (evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        break;

    case BLE_GAP_EVT_DISCONNECTED:
          sd_ble_gap_adv_start(&ble_adv_params);
          break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
          do { const uint32_t LOCAL_ERR_CODE = (sd_ble_gap_sec_params_reply(evt->evt . gap_evt . conn_handle, 0x00, &ble_gap_bond_params, 0)); if (LOCAL_ERR_CODE != ((0x0) + 0)) { do { app_error_handler((LOCAL_ERR_CODE), 0, 0); } while (0); } } while (0);

          break;

    case BLE_GAP_EVT_CONN_SEC_UPDATE:
        break;
      
    case BLE_GAP_EVT_AUTH_STATUS:
        break;

    default:
        break;
    }
}



 

void nrf_adv_conn_init(void)
{
    uint32_t error_code;
    int8_t        tx_power_level = 4;
    
     
    uint8_t flags = (0x04);


    memset(&ble_adv_data, 0, sizeof(ble_adv_data));

    ble_adv_data.flags = flags;
    ble_adv_data.name_type    = BLE_ADVDATA_FULL_NAME;
    ble_adv_data.p_tx_power_level = &tx_power_level;
    

    ble_gap_conn_sec_mode_t name_sec_mode = {1, 1};
    ble_gap_addr_t my_addr;
    
    error_code = sd_ble_gap_address_get(&my_addr);
    do { const uint32_t LOCAL_ERR_CODE = (error_code); if (LOCAL_ERR_CODE != ((0x0) + 0)) { do { app_error_handler((LOCAL_ERR_CODE), 0, 0); } while (0); } } while (0);
    
    char name[64];
    sprintf(name, "DEN RGB DEMO 1 #%02x%02x", my_addr.addr[5], my_addr.addr[4]);
        
    
    error_code = sd_ble_gap_device_name_set(&name_sec_mode, (uint8_t*) name, strlen(name));
    do { const uint32_t LOCAL_ERR_CODE = (error_code); if (LOCAL_ERR_CODE != ((0x0) + 0)) { do { app_error_handler((LOCAL_ERR_CODE), 0, 0); } while (0); } } while (0);
    
     
    error_code = ble_advdata_set(&ble_adv_data, 0);
    do { const uint32_t LOCAL_ERR_CODE = (error_code); if (LOCAL_ERR_CODE != ((0x0) + 0)) { do { app_error_handler((LOCAL_ERR_CODE), 0, 0); } while (0); } } while (0);

     
    error_code = sd_ble_gap_adv_start(&ble_adv_params);
    do { const uint32_t LOCAL_ERR_CODE = (error_code); if (LOCAL_ERR_CODE != ((0x0) + 0)) { do { app_error_handler((LOCAL_ERR_CODE), 0, 0); } while (0); } } while (0);
}


void nrf_adv_conn_evt_handler(ble_evt_t* evt)
{
    if (rbc_mesh_ble_evt_handler(evt) == ((0x0) + 0))
    {
        
        switch (evt->header.evt_id & 0xF0)
        {
        case 0x10:
            ble_gap_event_handler(evt);
            break;

        case 0x50:
            ble_gatts_event_handler(evt);
            break;

        default:
            break;
        }
    }
}
