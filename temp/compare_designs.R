
seafet_calib <- read_csv(here("temp", "data", "jan2024", "SeaFET2-0002300-Data-20240125T211850-TRIS.csv")) %>%
  select(2, 5, 6, 12) %>%
  rename(date_time=1, fet_ph=3, tris_ph = 4) %>%
  slice(50:n()) %>%
  slice(1:(n()-150)) %>%
  mutate(date_time=mdy_hm(date_time)) %>%
  #only keep the last 50 datapoints
  slice((n()-40):n())

#plot the data
ggplot(seafet_calib, aes(x=date_time, y=fet_ph)) +
  geom_point() +
  labs(x="pH_FET", y="pH_TRIS") +
  theme_bw()

# Calculate the linear model
model <- lm(seafet_calib$tris_ph ~ seafet_calib$fet_ph)
# Extract slope and intercept
slope <- coef(model)[2]
intercept <- coef(model)[1]

seafet <- read_csv(here("temp", "data", "jan2024", "SeaFET2-0002300-Data-20240125T200951.csv")) %>%
  select(2, 5, 6) %>%
  rename(date_time=1, p_h_fet=3) %>%
  mutate(date_time=mdy_hms(date_time),
         adjusted_ph = slope * p_h_fet + intercept)

arduino <- read_csv(here("temp", "data", "jan2024", "FINAL_arduino.csv")) %>%
  mutate(date_time=ymd_hms(date_time)) %>%
  select(date_time, p_h) %>%
  mutate(date_time = date_time - 1)

arduino <- read_csv(here("temp", "data", "jan2024", "0125tris_clean_omegas.csv")) %>%
  mutate(date_time=ymd_hms(trisdata.date_time),
         )

  select(date_time, p_h) %>%
  mutate(date_time = date_time - 1)

# Merge the arduino and seafet data frames
all_data <- full_join(arduino, seafet, by="date_time") %>%
  mutate(date_time=as_datetime(date_time)) %>%
  arrange(date_time) %>%
  filter(date_time > ymd_hms("2024-01-23 18:30:00"),
         date_time < ymd_hms("2024-01-25 11:30:00"))

# ggplot the data, p_h and p_h_fet over time and shown in two different colors
ggplot(all_data, aes(x=date_time)) +
  geom_point(aes(y=p_h, color="pH")) +
  geom_point(aes(y=p_h_fet, color="pH_FET")) +
  geom_point(aes(y=adjusted_ph, color="FET_adjusted")) +
  scale_color_manual(name="pH", values=c("pH"="blue", "pH_FET"="red", "FET_adjusted" = "green")) +
  labs(x="Date Time", y="pH") +
  theme_bw()

