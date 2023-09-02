# Installing Ubuntu 22.04 on VirtualBox

This section provides step-by-step instructions on how to install Ubuntu 22.04 as a virtual machine using Oracle VirtualBox. We will also cover how to access the terminal within the virtual machine.

## Prerequisites

Before you begin, make sure you have the following:

- [VirtualBox](https://www.virtualbox.org/) installed on your host system.
- The Ubuntu 22.04 ISO image downloaded from the [official Ubuntu website](https://ubuntu.com/download).

## Installation Steps

1. **Open VirtualBox:** Launch VirtualBox on your host system.

2. **Create a New Virtual Machine:**
   
   - Click the "New" button to create a new virtual machine.
   - Enter a name for your virtual machine (e.g., "Ubuntu 22.04").
   - Select "Linux" as the Type and "Ubuntu (64-bit)" as the Version.

3. **Configure Memory and Hard Disk:**

   - Allocate memory (RAM) to the virtual machine. A minimum of 2GB is recommended.
   - Create a new virtual hard disk with a size of at least 20GB. Choose the format type that suits your needs.

4. **Install Ubuntu:**

   - Select your newly created virtual machine in the VirtualBox Manager.
   - Click "Settings" and navigate to the "Storage" section.
   - Under "Controller: IDE," click the empty CD/DVD drive and select "Choose Virtual Optical Disk File."
   - Browse and select the Ubuntu 22.04 ISO image you downloaded.
   - Click "OK" to save the settings.

5. **Start the Virtual Machine:**

   - Back in the VirtualBox Manager, select your virtual machine.
   - Click the "Start" button to launch the virtual machine.
   - The Ubuntu installer should boot from the ISO image.

6. **Install Ubuntu:**

   - Follow the on-screen instructions to install Ubuntu 22.04.
   - You will be prompted to create a user account and set up system preferences.

7. **Accessing the Terminal:**

   - Once Ubuntu is installed and running, you can open the terminal by pressing `Ctrl` + `Alt` + `T` or searching for "Terminal" in the applications menu.
   - I personally faced some problems concerning opening the terminal, so I opened the TTY and installed a new terminal emulator in Ubuntu 22.04 (or any recent version of Ubuntu)
     by using the package manager commands and then launching the terminator and it worked just fine!

8. **Run Commands:**

   - You can now use the terminal to run commands, install software, and perform various tasks.
   - I will provide below some of the commands I tried.

## Additional Information

For more detailed information on installing Ubuntu 22.04, refer to the official [Ubuntu Installation Guide](https://help.ubuntu.com/community/Installation).

If you encounter any issues during the installation process or have specific questions about Ubuntu 22.04, feel free to ask for assistance.
## **Trial commands:**
![image](https://github.com/asserelzeki6/M.I.A_Training/assets/121148855/7a7576a2-d24a-4e73-bae3-3871c8e5cb73)


