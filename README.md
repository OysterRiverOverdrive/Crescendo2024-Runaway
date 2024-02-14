# Overdrive-Bot-2024
Robot Code for FIRST Robotics Competition Year 2024

## Developers

* Code changes are expected to be made through pull requests from a feature branch to `main`.
* Pull requests run automated sanity checks on the proposed changes using [git workflows]](.github/workflows).
* Pull requests are expected to be reviewed by a teammate before merging.

## Contribute

* Pull requests are required for all changes to `main`.
* Readable code is a priority, run `spotless` to ensure the
  code you are changing is formatted correctly.
  * Open the terminal window with CTRL+`
  * Enter `./gradlew spotlessApply`
  * Any changes should be included in your committed code.

## Common Issues

### Pull request `checks have failed`

If your pull request is blocked by a message like `All checks have failed` and `Merging is blocked`:

* Open the pull request in the browser.
* Click on "Details" which should bring you to the output of the failed check to figure out what needs to be fixed on your branch.

### How to Run Spotless

If you see `Execution failed for task ':spotlessJavaCheck'` then that means that your code is incorrectly formatted.
Spotless is integrated into our code to make sure that it remains organized.
To run spotless.
* In VSCode, Press CTRL+` to open the terminal at the bottom of the screen
* Click into the terminal
* Type `./gradlew spotlessApply` and press enter

It should say `Build Successful` which does not mean the code will build normally, however it was correctly formatted



### Running gradle commands

To run a command like `./gradlew spotlessApply`,

* From the WPI editor:
* Press CTRL+` which will open the terminal in the project.
* Type the gradle command and press enter.
