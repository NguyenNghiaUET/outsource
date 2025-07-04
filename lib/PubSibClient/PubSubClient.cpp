/*
  PubSubClient.cpp - A simple client for MQTT.
  Nick O'Leary
  http://knolleary.net
*/

#include "PubSubClient.h"
#include "Arduino.h"

PubSubClient::PubSubClient() {
    this->_state = MQTT_DISCONNECTED;
    this->_client = NULL;
    this->stream = NULL;
    this->keepAlive = 30; // Default keep-alive interval
    setCallback(NULL);
    this->isACK = true;
    this->failLength = -1;
}

PubSubClient::PubSubClient(Client& client) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setClient(client);
    this->stream = NULL;
    this->failLength = -1;
}

PubSubClient::PubSubClient(IPAddress addr, uint16_t port, Client& client) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(addr, port);
    setClient(client);
    this->stream = NULL;
    this->failLength = -1;
}
PubSubClient::PubSubClient(IPAddress addr, uint16_t port, Client& client, Stream& stream) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(addr,port);
    setClient(client);
    setStream(stream);
    this->failLength = -1;
}
PubSubClient::PubSubClient(IPAddress addr, uint16_t port, MQTT_CALLBACK_SIGNATURE, Client& client) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(addr, port);
    setCallback(callback);
    setClient(client);
    this->stream = NULL;
    this->failLength = -1;
}
PubSubClient::PubSubClient(IPAddress addr, uint16_t port, MQTT_CALLBACK_SIGNATURE, Client& client, Stream& stream) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(addr,port);
    setCallback(callback);
    setClient(client);
    setStream(stream);
    this->failLength = -1;
}

PubSubClient::PubSubClient(uint8_t *ip, uint16_t port, Client& client) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(ip, port);
    setClient(client);
    this->stream = NULL;
    this->failLength = -1;
}
PubSubClient::PubSubClient(uint8_t *ip, uint16_t port, Client& client, Stream& stream) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(ip,port);
    setClient(client);
    setStream(stream);
    this->failLength = -1;
}
PubSubClient::PubSubClient(uint8_t *ip, uint16_t port, MQTT_CALLBACK_SIGNATURE, Client& client) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(ip, port);
    setCallback(callback);
    setClient(client);
    this->stream = NULL;
    this->failLength = -1;
}
PubSubClient::PubSubClient(uint8_t *ip, uint16_t port, MQTT_CALLBACK_SIGNATURE, Client& client, Stream& stream) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(ip,port);
    setCallback(callback);
    setClient(client);
    setStream(stream);
    this->failLength = -1;
}

PubSubClient::PubSubClient(const char* domain, uint16_t port, Client& client) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(domain,port);
    setClient(client);
    this->stream = NULL;
    this->failLength = -1;
}
PubSubClient::PubSubClient(const char* domain, uint16_t port, Client& client, Stream& stream) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(domain,port);
    setClient(client);
    setStream(stream);
    this->failLength = -1;
}
PubSubClient::PubSubClient(const char* domain, uint16_t port, MQTT_CALLBACK_SIGNATURE, Client& client) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(domain,port);
    setCallback(callback);
    setClient(client);
    this->stream = NULL;
    this->failLength = -1;
}
PubSubClient::PubSubClient(const char* domain, uint16_t port, MQTT_CALLBACK_SIGNATURE, Client& client, Stream& stream) {
    this->_state = MQTT_DISCONNECTED;
    this->keepAlive = 30; // Default keep-alive interval
    this->isACK = true;
    setServer(domain,port);
    setCallback(callback);
    setClient(client);
    setStream(stream);
    this->failLength = -1;
}

PubSubClient& PubSubClient::setKeepAlive(uint16_t keepAlive) {
    this->keepAlive = keepAlive;
    return *this;
}

boolean PubSubClient::connect(const char *id) {
    return connect(id,NULL,NULL,0,0,0,0);
}

boolean PubSubClient::connect(const char *id, const char *user, const char *pass) {
    return connect(id,user,pass,0,0,0,0);
}

boolean PubSubClient::connect(const char *id, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage) {
    return connect(id,NULL,NULL,willTopic,willQos,willRetain,willMessage);
}

boolean PubSubClient::connect(const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage) {
    if (!connected()) {
        int result = 0;

        if (domain != NULL) {
            result = _client->connect(this->domain, this->port);
        } else {
            result = _client->connect(this->ip, this->port);
        }
        if (result) {
            nextMsgId = 1;
            // Leave room in the buffer for header and variable length field
            uint16_t length = 5;
            unsigned int j;

#if MQTT_VERSION == MQTT_VERSION_3_1
            uint8_t d[9] = {0x00,0x06,'M','Q','I','s','d','p', MQTT_VERSION};
#define MQTT_HEADER_VERSION_LENGTH 9
#elif MQTT_VERSION == MQTT_VERSION_3_1_1
            uint8_t d[7] = {0x00,0x04,'M','Q','T','T',MQTT_VERSION};
#define MQTT_HEADER_VERSION_LENGTH 7
#endif
            for (j = 0;j<MQTT_HEADER_VERSION_LENGTH;j++) {
                buffer[length++] = d[j];
            }

            uint8_t v;
            if (willTopic) {
                v = 0x06|(willQos<<3)|(willRetain<<5);
            } else {
                v = 0x02;
            }

            if(user != NULL) {
                v = v|0x80;

                if(pass != NULL) {
                    v = v|(0x80>>1);
                }
            }

            buffer[length++] = v;

            buffer[length++] = (this->keepAlive >> 8);
            buffer[length++] = (this->keepAlive & 0xFF);
            length = writeString(id,buffer,length);
            if (willTopic) {
                length = writeString(willTopic,buffer,length);
                length = writeString(willMessage,buffer,length);
            }

            if(user != NULL) {
                length = writeString(user,buffer,length);
                if(pass != NULL) {
                    length = writeString(pass,buffer,length);
                }
            }

            write(MQTTCONNECT,buffer,length-5);

            lastInActivity = lastOutActivity = millis();

            while (!_client->available()) {
                unsigned long t = millis();
                if (t-lastInActivity >= ((int32_t) MQTT_SOCKET_TIMEOUT*1000UL)) {
                    _state = MQTT_CONNECTION_TIMEOUT;
                    _client->stop();
                    return false;
                }
            }
            uint8_t llen;
            uint16_t len = readPacket(&llen);

            if (len == 4) {
                if (buffer[3] == 0) {
                    lastInActivity = millis();
                    pingOutstanding = false;
                    _state = MQTT_CONNECTED;
                    return true;
                } else {
                    _state = buffer[3];
                }
            }
            _client->stop();
        } else {
            _state = MQTT_CONNECT_FAILED;
        }
        return false;
    }
    return true;
}

// reads a byte into result
boolean PubSubClient::readByte(uint8_t * result) {
   uint32_t previousMillis = millis();
   while(!_client->available()) {
     uint32_t currentMillis = millis();
     if(currentMillis - previousMillis >= ((int32_t) MQTT_SOCKET_TIMEOUT * 1000)){
       return false;
     }
   }
   *result = _client->read();
   return true;
}

// reads a byte into result[*index] and increments index
boolean PubSubClient::readByte(uint8_t * result, uint16_t * index){
  uint16_t current_index = *index;
  uint8_t * write_address = &(result[current_index]);
  if(readByte(write_address)){
    *index = current_index + 1;
    return true;
  }
  return false;
}

// reads an entire MQTT packet;
// put "remaining length" (see MQTT spec) in lengthLength;
// return packet size;
uint16_t PubSubClient::readPacket(uint8_t* lengthLength) {
	/* PACKET STRUCTURE:
	1 byte: fixed header
	lengthLength bytes: remaining length (variable header size + payload size)
	2 bytes: topic length (if MQTTPUBLISH)
	n bytes: topic (if MQTTPUBLISH)
	2 bytes: message id (if qos>0)
	p byte; payload
	*/
	
    uint16_t len = 0;
    if(!readByte(buffer, &len)) return 0;
    bool isPublish = (buffer[0]&0xF0) == MQTTPUBLISH;
    uint32_t multiplier = 1;
    uint16_t length = 0; // remaining length
    uint8_t digit = 0; // single byte buffer
    uint16_t skip = 0; // if MQTTPUBLISH, = topic length + message id length (if qos>0)
    uint8_t start = 0; // =2 if MQTTPUBLISH, to skip the two already read topic length bytes in the for loop

	// find the reamining leanght (lengthLength)
    do {
        if(!readByte(&digit)) return 0;
        buffer[len++] = digit;
        length += (digit & 127) * multiplier;
        multiplier *= 128;
    } while ((digit & 128) != 0);
    *lengthLength = len-1;

    if (isPublish) {
        // Read in topic length to calculate bytes to skip over for Stream writing
        if(!readByte(buffer, &len)) return 0;
        if(!readByte(buffer, &len)) return 0;
        skip = (buffer[*lengthLength+1]<<8)+buffer[*lengthLength+2];
        start = 2;
        if (buffer[0]&MQTTQOS1) {
            // skip message id
            skip += 2;
        }
    }

	// read variable header and payload
    for (uint16_t i = start;i<length;i++) {
        if(!readByte(&digit)) return 0;
        if (this->stream) {
            if (isPublish && len-*lengthLength-2>skip) {
                this->stream->write(digit);
            }
        }
        if (len < MQTT_MAX_PACKET_SIZE) {
            buffer[len] = digit;
        }
        len++;
    }

    if (!this->stream && len > MQTT_MAX_PACKET_SIZE) {
        len = 0; // This will cause the packet to be ignored.
    }

    return len;
	
}

boolean PubSubClient::loop() {
    if (connected()) {
        unsigned long t = millis();
        if ((t - lastInActivity > this->keepAlive*1000UL) || (t - lastOutActivity > this->keepAlive*1000UL)) {
            if (pingOutstanding) {
                this->_state = MQTT_CONNECTION_TIMEOUT;
                _client->stop();
                return false;
            } else {
                buffer[0] = MQTTPINGREQ;
                buffer[1] = 0;
                _client->write(buffer,2);
                lastOutActivity = t;
                lastInActivity = t;
                pingOutstanding = true;
            }
        }
        while (_client->available()) {
            uint8_t llen;
            uint16_t len = readPacket(&llen);
            uint16_t msgId = 0;
            uint8_t *payload;
            if (len > 0) {
                lastInActivity = t;
                uint8_t type = buffer[0]&0xF0;
                if (type == MQTTPUBLISH) {
                    if (callback) {
                        uint16_t tl = (buffer[llen+1]<<8)+buffer[llen+2];
                        char topic[tl+1];
                        for (uint16_t i=0;i<tl;i++) {
                            topic[i] = buffer[llen+3+i];
                        }
                        topic[tl] = 0;
                        // msgId only present for QOS>0
                        if ((buffer[0]&0x06) == MQTTQOS1) {
                            msgId = (buffer[llen+3+tl]<<8)+buffer[llen+3+tl+1];
                            payload = buffer+llen+3+tl+2;
                            callback(topic,payload,len-llen-3-tl-2);

                            buffer[0] = MQTTPUBACK;
                            buffer[1] = 2;
                            buffer[2] = (msgId >> 8);
                            buffer[3] = (msgId & 0xFF);
                            _client->write(buffer,4);
                            lastOutActivity = t;

                        } else {
                            payload = buffer+llen+3+tl;
                            callback(topic,payload,len-llen-3-tl);
                        }
                    }
                } else if (type == MQTTPUBACK && this->pendingLength > 0) {
                    Serial.println("[PubSubClient] Received PUBACK");
                    Serial.println("Thời gian nhận PUBACK: ");
                    Serial.println(millis());
                    this->isACK = true;
                    this->pendingLength = -1;
                } 
                else if (type == MQTTPUBREC) {
                    buffer[0] = MQTTPUBREL | 2; // 7th bit of first byte must be set for PUBREL
                    _client->write(buffer,4); // 2nd, 3rd and 4th bytes of PUBREL are the same of PUBREC
                } else if (type == MQTTPINGREQ) {
                    buffer[0] = MQTTPINGRESP;
                    buffer[1] = 0;
                    _client->write(buffer,2);
                } else if (type == MQTTPINGRESP) {
                    pingOutstanding = false;
                }
            }
        }
        if (millis() - this->lastACKTime > ACK_TIMEOUT && this->pendingLength > 0) {
            this->isACK = false;
            this->failLength = this->pendingLength;
            for (uint16_t i = 0; i < this->pendingLength; i++) {
                this->failPayload[i] = this->pendingPayload[i];
            }
            this->pendingLength = -1;
        }
        return true;
    }
    return false;
}

boolean PubSubClient::publish(const char* topic, const char* payload) {
    return publish(topic,(const uint8_t*)payload,strlen(payload),0,false);
}

boolean PubSubClient::publish(const char* topic, const char* payload, uint8_t qos, boolean retained) {
    return publish(topic,(const uint8_t*)payload,strlen(payload),qos,retained);
}

boolean PubSubClient::publish(const char* topic, const uint8_t* payload, unsigned int plength) {
    return publish(topic, payload, plength, 0, false);
}

boolean PubSubClient::publish(const char* topic, const uint8_t* payload, unsigned int plength, uint8_t qos, boolean retained) {
	loop();
    if (connected()) {
        if (MQTT_MAX_PACKET_SIZE < 5 + 2+strlen(topic) + plength + (qos ? 2 : 0)) {
            // Too long
            return false;
        }
        // Leave room in the buffer for header and variable length field
        uint16_t length = 5;
        length = writeString(topic,buffer,length);
		if (qos) {
			nextMsgId++;
			if (nextMsgId == 0)
				nextMsgId = 1;
			buffer[length++] = (nextMsgId >> 8);
			buffer[length++] = (nextMsgId & 0xFF);
            this->isACK = false;
            this->pendingLength = plength;
            this->lastACKTime = millis();
            for (uint16_t i = 0; i < plength; i++) {
                this->pendingPayload[i] = payload[i];
            }
		}
        uint16_t i;
        for (i=0;i<plength;i++) {
            buffer[length++] = payload[i];
        }
        uint8_t header = MQTTPUBLISH;
        if (retained) {
            header |= 1;
        }
		header |= qos << 1;
        return write(header,buffer,length-5);

    }
    return false;
}
boolean PubSubClient::publish_P(const char* topic, const uint8_t* payload, unsigned int plength, uint8_t qos, boolean retained) {
    loop();

    Serial.print("[PubSubClient] Publishing to topic: ");
    Serial.println(topic);
    Serial.print("[PubSubClient] QoS: ");
    Serial.println(qos);
    Serial.print("[PubSubClient] Retained: ");
    Serial.println(retained ? "true" : "false");

    if (!connected()) {
        Serial.println("[PubSubClient] Not connected, publish failed!");
        return false;
    }

    uint8_t llen = 0;
    uint8_t digit;
    unsigned int rc = 0;
    uint16_t tlen;
    unsigned int pos = 0;
    unsigned int i;
    uint8_t header;
    unsigned int len;

    tlen = strlen(topic);

    header = MQTTPUBLISH;
    if (retained) {
        header |= 1;
    }
    header |= qos << 1;
    Serial.print("[PubSubClient] MQTT Header (after QoS): 0x");
    Serial.println(header, HEX);

    buffer[pos++] = header;
    len = plength + 2 + tlen + (qos > 0 ? 2 : 0); // Include message ID for QoS > 0
    do {
        digit = len % 128;
        len = len / 128;
        if (len > 0) {
            digit |= 0x80;
        }
        buffer[pos++] = digit;
        llen++;
    } while (len > 0);

    pos = writeString(topic, buffer, pos);

    if (qos > 0) {
        nextMsgId++;
        if (nextMsgId == 0) {
            nextMsgId = 1;
        }
        buffer[pos++] = (nextMsgId >> 8);
        buffer[pos++] = (nextMsgId & 0xFF);
        Serial.print("[PubSubClient] Message ID for QoS: ");
        Serial.println(nextMsgId);
    }

    Serial.print("[PubSubClient] Writing header and topic, bytes written: ");
    rc += _client->write(buffer, pos);
    Serial.println(rc);

    for (i = 0; i < plength; i++) {
        rc += _client->write((char)pgm_read_byte_near(payload + i));
    }

    Serial.print("[PubSubClient] Total bytes written: ");
    Serial.println(rc);
    Serial.print("[PubSubClient] Expected bytes: ");
    Serial.println(tlen + 4 + plength + (qos > 0 ? 2 : 0));

    lastOutActivity = millis();

    bool success = rc == tlen + 4 + plength + (qos > 0 ? 2 : 0);
    Serial.print("[PubSubClient] Publish ");
    Serial.println(success ? "successful" : "failed");

    return success;
}

boolean PubSubClient::write(uint8_t header, uint8_t* buf, uint16_t length) {
    uint8_t lenBuf[4];
    uint8_t llen = 0;
    uint8_t digit;
    uint8_t pos = 0;
    uint16_t rc;
    uint16_t len = length;
    do {
        digit = len % 128;
        len = len / 128;
        if (len > 0) {
            digit |= 0x80;
        }
        lenBuf[pos++] = digit;
        llen++;
    } while(len>0);

    buf[4-llen] = header;
    for (int i=0;i<llen;i++) {
        buf[5-llen+i] = lenBuf[i];
    }

#ifdef MQTT_MAX_TRANSFER_SIZE
    uint8_t* writeBuf = buf+(4-llen);
    uint16_t bytesRemaining = length+1+llen;  //Match the length type
    uint8_t bytesToWrite;
    boolean result = true;
    while((bytesRemaining > 0) && result) {
        bytesToWrite = (bytesRemaining > MQTT_MAX_TRANSFER_SIZE)?MQTT_MAX_TRANSFER_SIZE:bytesRemaining;
        rc = _client->write(writeBuf,bytesToWrite);
        result = (rc == bytesToWrite);
        bytesRemaining -= rc;
        writeBuf += rc;
    }
    return result;
#else
    rc = _client->write(buf+(4-llen),length+1+llen);
    lastOutActivity = millis();
    return (rc == 1+llen+length);
#endif
}

boolean PubSubClient::subscribe(const char* topic) {
    return subscribe(topic, 0);
}

boolean PubSubClient::subscribe(const char* topic, uint8_t qos) {
	loop();
    if (qos < 0 || qos > 1) {
        return false;
    }
    if (MQTT_MAX_PACKET_SIZE < 9 + strlen(topic)) {
        // Too long
        return false;
    }
    if (connected()) {
        // Leave room in the buffer for header and variable length field
        uint16_t length = 5;
        nextMsgId++;
        if (nextMsgId == 0) {
            nextMsgId = 1;
        }
        buffer[length++] = (nextMsgId >> 8);
        buffer[length++] = (nextMsgId & 0xFF);
        length = writeString((char*)topic, buffer,length);
        buffer[length++] = qos;
		return write(MQTTSUBSCRIBE|MQTTQOS1,buffer,length-5);
    }
    return false;
}

boolean PubSubClient::unsubscribe(const char* topic) {
	loop();
    if (MQTT_MAX_PACKET_SIZE < 9 + strlen(topic)) {
        // Too long
        return false;
    }
    if (connected()) {
        uint16_t length = 5;
        nextMsgId++;
        if (nextMsgId == 0) {
            nextMsgId = 1;
        }
        buffer[length++] = (nextMsgId >> 8);
        buffer[length++] = (nextMsgId & 0xFF);
        length = writeString(topic, buffer,length);
		return write(MQTTUNSUBSCRIBE|MQTTQOS1,buffer,length-5);
    }
    return false;
}

void PubSubClient::disconnect() {
    buffer[0] = MQTTDISCONNECT;
    buffer[1] = 0;
    _client->write(buffer,2);
    _state = MQTT_DISCONNECTED;
    _client->stop();
    lastInActivity = lastOutActivity = millis();
}

uint16_t PubSubClient::writeString(const char* string, uint8_t* buf, uint16_t pos) {
    const char* idp = string;
    uint16_t i = 0;
    pos += 2;
    while (*idp) {
        buf[pos++] = *idp++;
        i++;
    }
    buf[pos-i-2] = (i >> 8);
    buf[pos-i-1] = (i & 0xFF);
    return pos;
}

boolean PubSubClient::connected() {
    boolean rc;
    if (_client == NULL ) {
        rc = false;
    } else {
        rc = (int)_client->connected();
        if (!rc) {
            if (this->_state == MQTT_CONNECTED) {
                this->_state = MQTT_CONNECTION_LOST;
                _client->flush();
                _client->stop();
            }
        }
    }
    return rc;
}

PubSubClient& PubSubClient::setServer(uint8_t * ip, uint16_t port) {
    IPAddress addr(ip[0],ip[1],ip[2],ip[3]);
    return setServer(addr,port);
}

PubSubClient& PubSubClient::setServer(IPAddress ip, uint16_t port) {
    this->ip = ip;
    this->port = port;
    this->domain = NULL;
    return *this;
}

PubSubClient& PubSubClient::setServer(const char * domain, uint16_t port) {
    this->domain = domain;
    this->port = port;
    return *this;
}

PubSubClient& PubSubClient::setCallback(void(*callback)(char*,uint8_t*,unsigned int)){
    this->callback = callback;
    return *this;
}

PubSubClient& PubSubClient::setClient(Client& client){
    this->_client = &client;
    return *this;
}

PubSubClient& PubSubClient::setStream(Stream& stream){
    this->stream = &stream;
    return *this;
}

int PubSubClient::state() {
    return this->_state;
}