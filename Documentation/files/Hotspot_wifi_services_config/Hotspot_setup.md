# Setup Hotspot

Connect USB-c to GPRS Hotspot controlled by a raspberrypi4
- SD del telefon:
    - Number: 624484300
    - PIN: 1234
- Compte DIGI (https://midigi.digimobil.es/login)
    - User: 624484300
    - Pass: Pepinillo80!
- The Hotspot access point created is:
    - SSID: rUBotics
    - Pass: rUBot123!

To configure the Hotspot you can connect to the raspberrypi with ethernet cable
- es creara una connexio ethernet nova (Eth6)
- Select "Connexions de xarxa", "Wifi", "Properties", "Uso compartido". **UNselect** "Permetre que usuaris d'altres xarxes es connectin a traves de la connexió a internet d'aquest equip". OK.
- Select "Connexions de xarxa", "Wifi", "Properties", "Uso compartido". **Select** "Permetre que usuaris d'altres xarxes es connectin a traves de la connexió a internet d'aquest equip". OK.
- Open Real VNC:
    - Identify the IP raspberry (192.168.137.3)
    - user: pi
    - Password: raspberrypi
- Connect the modem i wait 2on led (net) blinks
- execute in terminal:
````shell
sudo pppd call gprs&
````
- The last line will be:
````shell
Script /etc/ppp/ip-up finished (pid 2050), status = 0x0
````
- execute in terminal:
````shell
sudo route add default ppp0
sudo systemctl restart hostapd
sudo systemctl restart dnsmasq
sudo systemctl restart systemd-networkd
````
- The Bridge rUBotics is ready with IP: 192.168.4.1
- You have to unconnect the ethernet cable
