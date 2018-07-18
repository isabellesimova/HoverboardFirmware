# System Workbench Setup
This is the easiest way I could find to set up the project - set up a blank ST project and adding the files with virtual linking.
## Creating the Project
1) Click File > New > C Project
2) C Project
    1) Enter a Project Name
    2) Select Project type: Executable > Ac6 STM32 MCU Project
    3) Select Toolchains: Ac6 STM32 MCU GCC
    4) Click Next
3) Select Configurations
    1) Click Next (Both configurations should be checked)
4) Target Configuration
    1) Click "Create a new custom board"
        1) Keep Define new board
        2) Enter new board name: STM32F103
        3) Select board chip: STM32F1
        4) Select mcu: STM32F103RCTx
        5) Click OK
    2) Select the board you just made!
        1) Select Series: STM32F1
        2) Select Board: STM32F103 (or whatever was just named)
    3) Click Finish
## Adding the Files
1) Drag the "inc", "src", and "drivers" directory (from Finder, or whatever window you can view file directory structure) and drop them onto your project folder in SystemWorkbench
2) It'll prompt you to select how files and folders should be imported: "Link to files and recreate folder structure with virtual folders“
3) It'll prompt you to overwrite? - overwrite for both inc & src (you're overwriting empty directories)
4) Replace the startup file by dragging the startup\_stm32f103xe.s file into startup (via linking), and delete the existing startup_stm32.s file
5) Include drivers directory in build:
    1) Right click on the directory
    2) Go to Resource Configurations > Exclude from Build...
    3) Deselect All
    4) click OK
6) Exclude the drivers > cmsis directory
    1) Right click on the directory
    2) Go to Resource Configurations > Exclude from Build...
    3) Select All
    4) click OK
7) Exclude some of the hal files in drivers > stm32f1xx\_hal\_driver > src: 
    1) Select all the files
    2) Deselect adc\_ex, adc, cortex, dma, gpio\_ex, gpio, iwdg, rcc\_ex, rcc, tim\_ex, tim, uart, hal
    2) Go to Resource Configurations > Exclude from Build...
    3) Select All
    4) click OK
## Configuring the Properties
1) Right click on your project folder
2) Go to Properties
3) Go to C/C++ General > Paths and Symbols
4) Includes
    1) Click Add… 
    2) Pick the directories below via File System (and check Add to all configurations)
        - motor\_controller/inc
        - motor\_controller/drivers/stm32f1xx\_hal\_driver/inc
        - motor\_controller/drivers/cmsis/device/st/stm32f1xx/inc
        - motor\_controller/drivers/cmsis/inc
        * note if you try to add via workspace path, it doesn’t work since we do virtual linking
3) Symbols
    1) Click Add.
        1) Name: STM32F103xE; no value
    4) Click OK
    
# Done!
Now you should be able to build & debug & add breakpoints!

_Note that there might be some Symbol cannot be resolved errors, if this is the case, check to see in your Preferences > C/C++ > Indexer that "Index unused headers" is not checked. Even if it is, toggling it and applying it a few times might be the fix._
