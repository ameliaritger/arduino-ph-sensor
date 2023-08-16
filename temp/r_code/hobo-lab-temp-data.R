library(tidyverse)
library(here)

temp <- read_csv(here("data", "temp", "Apr8_lab.csv")) %>%
  select(2:3) %>%
  setNames(c("date_time", "temp")) %>%
  mutate(date_time=mdy_hms(date_time),
         logger = "old") #aapply lubridate to date/time column
#date=format(date_time, '%m/%d/%Y'), #create only date column
#time=format(date_time, '%H:%M:%S')) #create only time column

temp2 <- read_csv(here("data", "temp", "Apr8_lab_newhobo.csv")) %>%
  select(2:3) %>%
  setNames(c("date_time", "temp")) %>%
  mutate(date_time=mdy_hms(date_time),
         logger = "new") #aapply lubridate to date/time column

temp_all <- rbind(temp, temp2) %>%
  filter(date_time > ymd_hms("2023-04-08 14:00:00"),
         date_time < ymd_hms("2023-04-08 16:00:00"))

ggplot(temp_all, aes(x=date_time, y=temp)) +
  geom_point(aes(color = logger))

temp_sub <- temp %>%
  filter(date_time < ymd_hms("2023-04-03 12:00:00"),
         date_time > ymd_hms("2023-04-03 10:00:00"))
         #date_time > ymd_hms("2023-04-02 20:00:00"))

ggplot(temp_sub, aes(x=date_time, y=temp)) +
  geom_point()
