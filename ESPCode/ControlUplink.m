
u = udp('192.168.4.1',9999);
fopen(u);

i = 0;
while true
    fwrite(u, 'Hello World!')

end
