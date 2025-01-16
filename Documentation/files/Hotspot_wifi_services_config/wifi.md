# Adding Wi-Fi connections

- Add and Connect to the desired Wi-Fi Network on startup (i.e. "rUBotics" Network)
````shell
sudo nmcli connection add type wifi ifname wlan0 con-name "rUBotics" ssid "rUBotics"
sudo nmcli connection modify "rUBotics" wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify "rUBotics" wifi-sec.psk "rUBot"
sudo nmcli connection modify "rUBotics" connection.autoconnect yes
sudo nmcli connection modify "rUBotics" connection.autoconnect-priority 40
````
- Add and Connect to the desired Wi-Fi Network on startup (i.e. "Manel" Network)
````shell
sudo nmcli connection add type wifi ifname wlan0 con-name "Manel" ssid "Manel"
sudo nmcli connection modify "Manel" wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify "Manel" wifi-sec.psk "manel123"
sudo nmcli connection modify "Manel" connection.autoconnect yes
sudo nmcli connection modify "Manel" connection.autoconnect-priority 30
````
- Add and Connect to the desired Wi-Fi Network on startup (i.e. "N15L" Network)
````shell
sudo nmcli connection add type wifi ifname wlan0 con-name "N15L" ssid "N15L"
sudo nmcli connection modify "N15L" wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify "N15L" wifi-sec.psk "labepc1234"
sudo nmcli connection modify "N15L" connection.autoconnect yes
sudo nmcli connection modify "N15L" connection.autoconnect-priority 20
````
- Add and Connect to the desired Wi-Fi Network on startup (i.e. "STRONG_ATRIA_AY4U_5G" Network)
````shell
sudo nmcli connection add type wifi ifname wlan0 con-name "Casa" ssid "STRONG_ATRIA_AY4U_5G"
sudo nmcli connection modify "Casa" wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify "Casa" wifi-sec.psk "3HCG3ykAUc"
sudo nmcli connection modify "Casa" connection.autoconnect yes
sudo nmcli connection modify "Casa" connection.autoconnect-priority 10
````
