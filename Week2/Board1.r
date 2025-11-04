# Load necessary library
library(openxlsx)

# read in the data
board1 <- read.xlsx("Week2/wk 2 WS1 exam data(2).xlsx",
                    sheet = 1, startRow = 3, )

# Display dataset
View(board1)

# Display Summary statistics
print(summary(board1))


