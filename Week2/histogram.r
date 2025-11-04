library(openxlsx)

# read in the data
board2 <- read.xlsx("Week2/wk 2 WS1 exam data(2).xlsx",
                    sheet = 'data2', startRow = 3)

# Plot histogram
hist(board2$`%`, 
     main = "Distribution of Total Scores",
     xlab = "Total Score", 
     ylab = "Frequency",
     col = "lightgreen",
     breaks = 9)  # Number of bins