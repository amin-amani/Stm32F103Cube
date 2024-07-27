# IWDG


<a href=""><img src="https://github.com/amin-amani/Stm32F103Cube/blob/master/Watchdog/iwdg3.jpeg" alt="LPIC1 ||" width="150"/> 


## Let’s take an example, if i want to set the Timeout of 20 ms.

I decided to choose the Prescalar as 8. So PR = 1

If we substitute these values in the above formula, we will get the RL = 79.

So we need to substitute counter value = 79 in our setup.

If the RL value goes higher than 4095, then you need to increase the prescalar.
