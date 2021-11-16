

# This article describes how to connect T-Impulse to TTN V3
### necessity:
- A gateway that can connect to TTN V3
- One T-Impulse
- One Type-C cable
- A TTN account

## Step
1. Open the TTN website, establish a gateway, and connect the gateway to the TTN server [(refer to the construction of the gateway)](https://github.com/Xinyuan-LilyGO/SX1302_HAL_GATEWAY)
![](img/10.png)

2. Create a new Applications
![](img/1.png)
![](img/2.png)

3. Create a new terminal device
![](img/3.png)
![](img/4.png)
![](img/5.png)

4. Remember the three fields AppEUI, DevEUI, AppKey,
Where AppEUI and DevEUI are in LSB format, and AppKey is in MSB format
![](img/6.png)

5. Copy the three fields into the example program loramac.cpp array
![](img/7.png)

6. Set T-Impulse to DFU mode (first press and hold the BOOT button, then press the RST button, release RST and then release the BOOT button), if you have any questions, [(please refer to here)](https://github.com/Xinyuan-LilyGO/LilyGo-T-Impulse)

7. Click platfromIO to start burning, the following picture is displayed, which means the upload is successful
![](img/8.png)
![](img/9.png)

8. After the upload is successful, you can see T-Impulse apply to join at the gateway
![](img/11.png)

9. T-Impulse has been successfully connected to TTN V3 so far
## Cayenne example
Based on the above steps, forwarding the device to cayenne requires:
  - cayenne account

  ### Step:
  1. At present, a small part of the Cayenne package has been integrated in the sample program, so just create a Cayenne device directly
  ![](img/12.png)
  ![](img/13.png)
  ![](img/14.png)

  2. Fill in the newly created terminal device DevEUI, here you don’t need to bother about lsb or msb,
  Click finish
   ![](img/15.png)
   ![](img/16.png)

3. Return to TTN, click Payload formatters, set uplink to CayenneLPP, click save,
![](img/17.png)
![](img/18.png)
![](img/19.png)

4. Create and select a hook to the Cayenne server, click save
![](img/20.png)
![](img/21.png)
![](img/22.png)

5. When this step is completed, wait for a while on the Cayenne page, the forwarded data will appear in the overview
![](img/23.png)

6. At present, all the basic construction has been completed. If the positioning is successful outdoors, the current position will be automatically refreshed