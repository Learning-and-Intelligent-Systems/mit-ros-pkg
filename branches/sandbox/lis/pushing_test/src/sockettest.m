close all; clear; clc;
import java.net.Socket
import java.io.*

host = 'localhost';
port = 50000;

input_socket = [];
message = [];
nMessages = 0;
while true
    try
        if isempty(input_socket)
            input_socket = Socket(host,port);
        end
        
        input_stream = input_socket.getInputStream;
        d_input_stream = DataInputStream(input_stream);
        
        bytes_available = input_stream.available;
        
        if bytes_available>0
            message = zeros(1, bytes_available, 'uint8');
            for i = 1:bytes_available
                message(i) = d_input_stream.readByte;
            end
            
            nMessages=nMessages+1;
            message = char(message)
            if (nMessages>9)
                input_socket.close;
                break;
            end
        end
        
    catch err
        err
    end
end

%input_socket.close;

