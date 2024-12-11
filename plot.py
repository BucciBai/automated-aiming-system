import matplotlib.pyplot as plt
import pandas as pd

# Create a DataFrame for the metrics comparison
data = {
    "Metric": ["Tracking Accuracy (%)", "Response Time (s)", "Error Rate (%)"],
    "Original System": [90, 0.3, 5],
    "Updated System": [96, 0.6, 2]
}
df = pd.DataFrame(data)

# Generate a comparison bar chart
fig, ax = plt.subplots(figsize=(8, 5))

# Define positions and bar width
bar_width = 0.35
index = range(len(df["Metric"]))

# Plot bars
bars1 = ax.bar(index, df["Original System"], bar_width, label="Original System")
bars2 = ax.bar([i + bar_width for i in index], df["Updated System"], bar_width, label="Updated System")

# Annotate bars with their values
for bar in bars1:
    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
            f'{bar.get_height()}', ha='center', va='bottom')
for bar in bars2:
    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
            f'{bar.get_height()}', ha='center', va='bottom')

# Add labels, title, and legend
ax.set_xlabel("Metrics")
ax.set_ylabel("Values")
ax.set_title("Performance Comparison: Original vs Updated System")
ax.set_xticks([i + bar_width / 2 for i in index])
ax.set_xticklabels(df["Metric"], rotation=15)
ax.legend()

# Save the plot with annotations
chart_path = "performance_comparison_annotated_chart.png"
plt.tight_layout()
plt.savefig(chart_path)
plt.show()

# Print the path to the saved chart
print(f"Chart saved to {chart_path}")
