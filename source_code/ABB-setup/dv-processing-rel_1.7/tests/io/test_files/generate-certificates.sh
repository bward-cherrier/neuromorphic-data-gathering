#! /bin/bash

openssl genrsa -out ca-key 2048
openssl genrsa -out server-key 2048
openssl genrsa -out client-key 2048

openssl req -x509 -new -nodes -key ca-key -sha256 -days 365000 -out ca-cert -subj /C=US/ST=CA/L=Somewhere/O=Someone/CN=FoobarCA

openssl req -new -sha256 -key server-key -subj /C=US/ST=CA/L=Somewhere/O=Someone/CN=Foobar -out server.csr
openssl x509 -req -in server.csr -CA ca-cert -CAkey ca-key -CAcreateserial -out server-cert -days 365000 -sha256

openssl req -new -sha256 -key client-key -subj /C=US/ST=CA/L=Somewhere/O=Someone/CN=Foobar -out client.csr
openssl x509 -req -in client.csr -CA ca-cert -CAkey ca-key -CAcreateserial -out client-cert -days 365000 -sha256
