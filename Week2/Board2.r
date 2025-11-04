# Load necessary library
library(openxlsx)
install.packages("gplots") 
library(gplots)  # For heatmap.2

# read in the data
board2 <- read.xlsx("Week2/wk 2 WS1 exam data(2).xlsx",
                    sheet = 'data2', startRow = 3)

# Display dataset
View(board2)

# Display Summary statistics
print(summary(board2))



# Select numeric columns excluding % and ID for correlation analysis
numeric_cols <- sapply(board2, is.numeric)
cols_to_exclude <- c("%", "ID")
selected_cols <- names(board2)[numeric_cols & !names(board2) %in% cols_to_exclude]

# Calculate correlation matrix for selected columns only
cor_matrix <- cor(board2[, selected_cols], use = "complete.obs")

# Calculate R² (correlation squared)
r_squared_matrix <- cor_matrix^2

# Heatmap with key using heatmap.2
heatmap.2(r_squared_matrix, 
          main = "R² Between Variables (Excluding % and ID)",
          symm = TRUE,
          key = TRUE,
          key.title = "R² Values",
          trace = "none",  # Remove trace lines
          density.info = "none")  # Remove density plot from key