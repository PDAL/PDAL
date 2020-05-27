library(ggplot2)
library(reshape2)

reference <- c(2, 2, 1, 2, 5, 18, 51, 107, 166, 195, 176, 125, 70, 34, 14, 7, 5, 4, 5, 4, 3, 1, 0, 0)
low <- c(3, 3, 2, 1, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 4, 15, 50, 119, 194, 243, 255, 255, 217, 158, 97, 54, 30, 23, 20, 20, 18, 19, 20, 19, 17, 15, 13, 11, 12, 11, 10, 10, 10, 9, 10, 10, 10, 9, 11, 14, 27, 47, 66, 69, 54, 34, 19, 12, 9, 8, 7, 8, 12, 18, 28, 35, 34, 26, 16, 11, 15, 33, 64, 94, 105, 89, 58, 30, 14, 9, 7, 7, 7, 9, 9, 9, 7, 6, 5, 5, 6, 6, 6, 6, 5, 5, 4, 3, 4, 4)
high <- c(2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 3, 1, 1, 2, 4, 11, 29, 53, 74, 81, 73, 54, 34, 17, 8, 3, 3, 4, 4, 3, 2, 2, 2, 2, 2, 2, 3, 5, 4, 4, 3, 2, 1, 2, 1, 2, 1, 1, 2, 3, 4, 5, 6, 5, 4, 2, 3, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 3, 3, 4, 5, 4, 4, 3, 1, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 1)

df <- data.frame(reference=reference, time=seq(0, length(reference) - 1) * 1e-9)
ggplot(df, aes(time, reference)) +
    geom_line() +
    geom_point() +
    xlab("Time from the start of record in seconds") +
        ylab("Intensity")
    ggsave("images/reference-pulse.png")

time <- seq(0, length(low) - 1) * 1e-9
df <- data.frame(low=low, time=time)
peaks <- data.frame(time=c(time[21], time[54], time[66], time[75]),
                    amplitude=c(255, 69, 35, 105))
df.melt <- melt(df, id.vars="time", variable.name="Channel")

p <- ggplot(df.melt, aes(time, value)) +
    geom_point() +
    geom_line() +
    geom_point(aes(time, amplitude), peaks, colour = "red", shape = 8, size = 4) +
    xlab("Time from the start of record in seconds") +
    ylab("Intensity")
ggsave("images/return-pulse.png")
