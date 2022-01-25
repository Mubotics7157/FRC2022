package frc.util;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Pixy2Handler{
    boolean lampOn = false;
    byte[] lastCache = {
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00
    };
    byte[] localCache = {
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00,
        (byte) 0x00
    };
    //public ArrayList<String> VectorData = new ArrayList<String>();
    public final byte[] CHECKSUM_VERSIONREQUEST = {
        (byte) 0xae,  // first byte of no_checksum_sync (little endian -> least-significant byte first)
        (byte) 0xc1,  // second byte of no_checksum_sync
        (byte) 0x0e,  // this is the version request type
        (byte) 0x00   // data_length is 0
    };

    public final byte[] CHECKSUM_SETLEDCOLOR = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x14,
        (byte) 0x03,
        (byte) 0x00,
        (byte) 0xFF,
        (byte) 0x00
    };

    public final byte[] CHECKSUM_LAMPON = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x16,
        (byte) 0x02,
        (byte) 0x01,
        (byte) 0x01
    };

    public final byte[] CHECKSUM_LAMPOFF = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x16,
        (byte) 0x02,
        (byte) 0x00,
        (byte) 0x00
    };

    public final byte[] CHECKSUM_GETMAINFEATURES = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x30,
        (byte) 0x02,
        (byte) 0x00,
        (byte) 0x07
    };

    public final byte[] CHECKSUM_SETLINEMODE = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x02,
        (byte) 0x05,
        (byte) 0x6C,
        (byte) 0x69,
        (byte) 0x6E,
        (byte) 0x65,
        (byte) 0x00
    };

    public final byte[] CHECKSUM_GETBLOCKS = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x24,
        (byte) 0xFF,
        (byte) 0x01
    };

    public final byte[] CHECKSUM_SETMODE = {
        (byte) 0xae,
        (byte) 0xc1,
        (byte) 0x36,
        (byte) 0x01,
        (byte) 0x01
    };

    I2C pixy = new I2C(Port.kOnboard, 0x54);
    boolean vectorDetected = true;

    public void init(){
        pixy.writeBulk(CHECKSUM_SETLINEMODE);
        System.out.println("initializing pixy...");
    }

    public void toggleLamp(){
        if(lampOn){
            pixy.writeBulk(CHECKSUM_LAMPOFF);
            lampOn = false;
        }else{
            pixy.writeBulk(CHECKSUM_LAMPON);
            lampOn = true;
        }
    }

    public void sendRequest(byte[] byteArray){
        pixy.writeBulk(byteArray);
        byte[] initBuffer =  new byte[14];
        pixy.readOnly(initBuffer, 14);
        if(initBuffer[10] != -2 && initBuffer[10] != -128){
            vectorDetected = true;
            localCache = initBuffer;
            lastCache = initBuffer;
        }else{
            localCache = lastCache;
            vectorDetected = false;
        }
        
    }

    public void check(){

    }

    public void printLocalCache(){
        System.out.println("FEATURE-TYPE: " + localCache[6]);
        System.out.println("FEATURE-LENGTH: " + localCache[7]);
        System.out.println("x0: " + localCache[8]);
        System.out.println("y0: " + localCache[9]);
        System.out.println("x1: " + localCache[10]);
        System.out.println("y1: " + localCache[11]);
        System.out.println("index: " + localCache[12]);
        System.out.println("flags: " + localCache[13]);
        if((localCache[10]-localCache[8])!=0)
            System.out.println("angle: " + Math.atan((localCache[11]-localCache[9])/(localCache[10]-localCache[8]))*180/Math.PI);
    }

    public byte[] HandleInput()
    {
        pixy.writeBulk(CHECKSUM_GETMAINFEATURES);
        
        byte[] initBuffer = new byte[32];
        
        pixy.readOnly(initBuffer, 12);

        //test this thing
        for(int i = 0; i < 12; i++)
        {
            System.out.println(initBuffer[i]);
        }

        int bufferLength = (int) initBuffer[3];
        byte[] returnBuffer = new byte[bufferLength];

        //CONTRIBUTION BY DANIEL SHIM :)
        try{
            pixy.readOnly(returnBuffer, bufferLength);
        }catch(Exception e){
            System.out.println("No vectors found.");
            System.out.println(e.toString());
        }

        System.out.println("Pixy Data: ");
        //System.out.println(returnBuffer[6*8]);
        /*for(int i=returnBuffer.length -1; i>=0; i--){
            System.out.println(Byte.toString(returnBuffer[i]));
        }
            
        }*/
        for(int i = 0; i < Math.ceil(returnBuffer.length/8); i++)
        {
            System.out.println(returnBuffer);
            for(int j = 0; j < 8; j++)
            {
                try
                {
                    System.out.print(Integer.toHexString(returnBuffer[(i*8)+j] & 0xff) + ' ');
                    //VectorData.add(Integer.toHexString(returnBuffer[(i*8)+j] & 0xff) + ' ');
                    //System.out.print(returnBuffer[i*8+j]);
                    //System.out.print(Integer.toHexString(returnBuffer[8]));
                } catch (Exception e)
                {
                    
                }
                
                System.out.println();
            }
        }
        return returnBuffer;
    }
}