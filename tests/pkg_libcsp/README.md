## Libcsp tests

Use this program to test the CAN connection using libcsp

### Wiring

![wiring](https://os.mbed.com/media/uploads/hudakz/can_nucleo_hello.png)

### Communication

Once the two boards are wired, run `server` on one and `client` on the second to start the communication, you should see this output on server:
```
SERVICE: Ping received
csp_sys_reboot not supported - no user function set
Packet received on MY_SERVER_PORT: Hello World (33)
```

and on the client:
```
Ping address: 2, result 11 [mS]
reboot system request sent to address: 2
Ping address: 2, result 12 [mS]
reboot system request sent to address: 2
```

By default server is address 1 and client is address 2, and you can change the client with the optionnal arguments:
```
client <address> <can_device>
```