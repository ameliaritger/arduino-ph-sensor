library(tidyverse)
library(janitor)
library(here)
library(lubridate)

seafet_calib <- read_csv(here("temp", "data", "jan2024", "SeaFET2-0002300-Data-20240125T211850-TRIS.csv")) %>%
  select(2, 5, 6, 12, 13) %>%
  rename(date_time=1, int_ph=3, tris_ph=4, ext_ph=2, tris_ext=5) %>%
  slice(10:n()) %>%
  #slice(1:(n()-150)) %>%
  mutate(date_time=mdy_hm(date_time))

seafet_calib_diff <- seafet_calib %>%
  mutate(diff_ext = tris_ext - ext_ph) %>%
  summarise(mean_diff_ext=mean(diff_ext))

  #slice((n()-40):n())
mean_diff_ext <- seafet_calib_diff$mean_diff_ext

# Calculate the linear model, Extract slope and intercept
#model <- lm(seafet_calib$int_ph ~ seafet_calib$tris_ph)
#slope <- coef(model)[2]
#intercept <- coef(model)[1]

seafet <- read_csv(here("temp", "data", "jan2024", "SeaFET2-0002300-Data-20240125T200951.csv")) %>%
  select(2, 5, 6, 9) %>%
  rename(date_time=1, int_ph=3, ext_ph=2, temp_seafet=4) %>%
  mutate(date_time=mdy_hms(date_time),
         ext_adjust = ext_ph + mean_diff_ext)

arduino <- read_csv(here("temp", "data", "jan2024", "FINAL_arduino.csv")) %>%
  mutate(date_time=ymd_hms(date_time),
         date_time = date_time - 1) %>%
  rename(date_time=1, ph=3, temp_ard=2)

# Merge the arduino and seafet data frames
all_data <- full_join(arduino, seafet, by="date_time") %>%
  mutate(date_time=as_datetime(date_time)) %>%
  arrange(date_time) %>%
  filter(date_time > ymd_hms("2024-01-23 18:30:00"),
         date_time < ymd_hms("2024-01-25 11:30:00")) %>%
  filter(!is.na(ph),
         !is.na(ext_adjust))

# ggplot the data, p_h and p_h_fet over time and shown in two different colors
ggplot(all_data, aes(x=date_time)) +
  geom_point(aes(y=ph, color="Arduino")) +
  geom_point(aes(y=ext_adjust, color="SeaFET")) +
  #create a colorblind friendly color palette using blue and orange
  scale_color_manual(values=c("#D55E00", "#0072B2")) +
  #change title of legend
  labs(x="Date Time", y="pH", color = "Sensor") +
  theme_bw()

# save the plot
ggsave(here("temp", "data", "jan2024", "compare_designs.png"), width=10, height=6, dpi=300)

# create summary statistics comparing the two sensors, after removing NA rows
summary_stats <- all_data %>%
  summarize(mean_ph_arduino = mean(p_h),
            mean_ph_ext = mean(ext_adjust),
            mean_diff = mean(p_h - ext_adjust),
            sd_ph_arduino = sd(p_h),
            sd_ph_ext = sd(ext_adjust),
            sd_diff = sd(p_h - ext_adjust),
            range_ph_arduino = max(p_h)-min(p_h),
            range_ph_seafet = max(ext_adjust)-min(ext_adjust))

write_csv(summary_stats, here("temp", "data", "jan2024", "summary_stats.csv"))


# Only keep 36 hours of data
all_data_36 <- all_data %>%
  filter(date_time > ymd_hms("2024-01-23 19:00:00"),
         date_time < ymd_hms("2024-01-25 07:00:00"))

summary_stats <- all_data_36 %>%
  summarize(mean_ph_arduino = mean(p_h),
            mean_ph_ext = mean(ext_adjust),
            mean_diff = mean(p_h - ext_adjust),
            sd_ph_arduino = sd(p_h),
            sd_ph_ext = sd(ext_adjust),
            sd_diff = sd(p_h - ext_adjust),
            range_ph_arduino = max(p_h)-min(p_h),
            range_ph_seafet = max(ext_adjust)-min(ext_adjust),
            var_ph_arduino = var(p_h),
            var_ph_ext = var(ext_adjust))

#save the summary statistics
write_csv(summary_stats, here("temp", "data", "jan2024", "summary_stats36.csv"))
