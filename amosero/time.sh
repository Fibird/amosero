#during timimng issues this script synchronise the time periodically
while true
 do
    sudo ntpdate ntp.ubuntu.com
    sleep 3
done
