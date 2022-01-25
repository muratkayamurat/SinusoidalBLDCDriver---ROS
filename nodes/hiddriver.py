from __future__ import print_function

import hid
import time
import array


class hiddriver():
    def __init__(self,desiredProduct=""):
        self.desiredProduct=desiredProduct
        self.productFound=False
        self.deviceReady=False
        for d in hid.enumerate():
            if(d["product_string"]==desiredProduct):
            #print(desiredProduct)
            #keys = list(d.keys())
            #keys.sort()
            #for key in keys:
                #print("%s : %s" % (key, d[key]))
                self.desiredProductVID=d["vendor_id"]
                self.desiredProductPID=d["product_id"]
                self.productFound = True
                break

        # try opening a device, then perform write and read
        if(self.productFound):
            try:
                self.h = hid.device()
                self.h.open(self.desiredProductVID,self.desiredProductPID) # TREZOR VendorID/ProductID
                self.deviceReady=True

                #print("Manufacturer: %s" % h.get_manufacturer_string())
                #print("Product: %s" % h.get_product_string())
                #print("Serial No: %s" % h.get_serial_number_string())

                # enable non-blocking mode
                self.h.set_nonblocking(0)

                # write some data to the device
            except Exception as exc:
                self.h = None
                print("Hid Driver Usage Error!!!",str(exc))
        else:
            self.h = None
            print(self.desiredProduct+" Not Found!!")

    def list_devices(self):
        for d in hid.enumerate():
            print(d)


    def __close__(self):
        if not(self.h==None):
            self.h.close()

    def __release__(self):
        if not(self.h==None):
            self.h.close()

    def __del__(self):
        if not(self.h==None):
            self.h.close()

    def close_device(self):
        if not(self.h==None):
            self.h.close()

    def read_device(self,length):
        readed=""
        if not(self.h==None):
            try:
                d = self.h.read(length)
                readed="".join(chr(x) for x in d)
            except Exception as exc:
                pass
                readed=""
                self.h = None
                self.deviceReady=False
                print("Hid Driver Connection Error!!!:%s",str(exc))
        # else:
        #     time.sleep(5)
        #     try:
        #         self.h = hid.device()
        #         self.h.open(self.desiredProductVID,self.desiredProductPID) # TREZOR VendorID/ProductID
        #         self.deviceReady=True
        #     except Exception as exc:
        #         self.h = None
        #         print (str(exc))

        return readed

    def write_device(self,command):
        sended=False
        if not(self.h==None):
            try:
                commandByte=array.array('B',command.encode())
                self.h.write(commandByte)
                sended=True
            except Exception as exc:
                pass
                print(str(exc))
        return sended
            

if __name__ == '__main__':
    RightMotor="SeomakBLDCRight"
    LeftMotor="SeomakBLDCLeft"
    RightMotorHid = hiddriver(RightMotor)
    print(RightMotorHid.list_devices())
    # LeftMotorHid = hiddriver(LeftMotor)
    # RightMotorHid.write_device("C=+000")
    # LeftMotorHid.write_device("C=+000")
    # while(True):
    print("Right Motor Speed="+RightMotorHid.read_device(64))
    #     print("Left Motor Speed="+LeftMotorHid.read_device(64))






    
