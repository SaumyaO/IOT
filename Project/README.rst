Project:

Steps to set up on Linux system an Andriod Phone:
***************
#. Download the entire folder under zephyrproject/zephyr

#. Go to the zephyr folder where the project is downloaded : cd ~/zephyrproject/zephyr

#. To Compile: west build -b reel_board_v2 Project  -- -DSHIELD=link_board_eth

#. To execute : west flash

#. On your phone, use the nRFConnect app to Scan for devices and look for "reel board"

#. Connect to the device.

#. check the reel_board it would display connected

#. CLick Generic Access

#. Click Device Name upward arrow : Enter the name ( This will be the name of the board)

#. Once the name is comfirm click send.
  
#. On the phone, the app would display to enter the passkey. On the reel_board, the passkey would be
   displayed.

#. Enter the passkey on phone. 

#. The board would reflect the Pairing is complete and would display the name of the board just entered.

#. Disconnect the board from the phone by clicking diconnect on the phone . 
   Check the reel_board would display that mesh started and would assign a unique address.

#. Press the UserButton back of the board to see various user interface 

To reset a board to its initial state (disable mesh, erase the stored
text, and make it connectable over GATT):

#. Keep the user button pressed when powering on (or press the reset button when powered)
#. Wait until "Reseting Device" is shown
