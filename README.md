# Payload Software 2024-2025

![Rocket Team Logo](https://images.squarespace-cdn.com/content/v1/60e52c1c82890c732f0f09ca/1625632748453-9PZFYMLWAH05UU4X9ZEK/Logo_F_RG.png)

This is the code for the *McGill Rocket Team*'s *Payload Software* project team.

**Please read this document carefully.**
If you reread this document again for clarification or to remember something, any **important instructions which should know to follow** will be written in *italics*.

# Folders

Each of the folders in the repository has a specific purpose.

## [Documentation](./documentation/)

A collection of mainly **Markdown** files with detailed explanations of things you will find useful while programming for Payload Software and STM32s specifically.
It also has some necessary drivers and reference code.

### [Documentation/Assets](./documentation/assets/)

A collection of images to be embedded in the **Markdown** files in the documentation folder.

## [Legacy](./legacy/)

The final state of the STM32 project used for last year's competition PCB. (The one that we put in the rocket for launch.)

*Do not edit any files in this folder.*

If you need to copy any code from any files in this folder, try to rewrite it to be as organized and modular as possible.

## [Main](./main/)

This folder contains the STM32CubeIDE project and its associated code which will be used on the final payload PCB.

## [R&D (RND)](./rnd/)

This folder will contain the STM32CubeIDE projects and other files used for testing and R&D. This is where proof of concept tests and testing of features will be conducted before they are implemented on the [main PCB](#main).

Each project should be contained within in its own subfolder in the R&D folder. *Subfolder names should be all lowercase with underscores (`_`) for spaces.*

*A separate STM32CubeIDE project should be made for each test/R&D effort in order to keep the code modular and readable. 
Once you're done and have completed the test, you should make a `README.md` file in the project root documenting what changing you made to the default project to achieve the desired result and explain any decisions you made.
It can also contain any links or resources you used during your testing.*

**Example:**
If you want to test a temperate sensor, make a new project for the Nucleo board that you're going to be use in a subfolder.
Name the project and the subfolder you put it in something like `temperature_sensor`.
Then, make a markdown file called `README.md` in the `temperature_sensor` folder.
If you write code in `main.c`, write about what you've written.
If you change any configurations (especially clock configurations) in the `.ioc` file, 
If you find any good tutorials or references, link pages or YouTube videos in the file.
If you downloaded/added any external code or drivers, put a link to where you got them and where they are currently implemented in the test.
Once you're satisfied with the test, [make a pull request](#branches) in the GitHub repository and the branch will be merged with main.

## [Scripts](./scripts/)

This folder contains smaller, usually self-contained programs which can be run as scripts.

These will mostly be used for analysis of the data gathered from the SD card during flight or during tests.

# Branches

## Main

This is where all of the finalized code and documentation will live. When things are done being written and tested, they will be merged into main.

## Making a Branch

*Whenever you start working on a new feature/test/R&D effort, you should make a new branch with a representative name. Branches should be named in all lowercase with dashes (`-`) for spaces.*

If you work on something other than an STM32 project like a script or documentation, as long as it doesn't affect too much, the work can be done in the main branch. Use your judgement, or ask a lead.

*Example:*
To test a new temperature sensor, make a new branch off of main called `temperature-sensor`. 

## Merging a Branch

Merging should be done through pull request. If you're done with what you're working on, you should make a pull request, which will then be reviewed and merged.

*If you're writing or implementing a feature to go on the Payload PCB, you should make a pull request once you're finished testing.*

*If you're working on something in [R&D](#rd-rnd), you should make a pull request once you're done with your test and the report on the test is written in the `README`.*
