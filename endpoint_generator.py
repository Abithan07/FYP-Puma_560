import csv
import random

# Configuration
num_path = 200
start_id = 1

# Define joint ranges
def get_random_j1():
    """Joint 1: (100 to 150) or (-100 to -150) deg"""
    choice = random.choice([1, -1])
    return random.randint(100, 150) * choice

def get_random_j2():
    """Joint 2: (15 to -60) deg"""
    return random.randint(-60, 15)

def get_random_j3():
    """Joint 3: (60 to 100) or (180 to 225) deg"""
    choice = random.choice([1, 2])
    if choice == 1:
        return random.randint(60, 100)
    else:
        return random.randint(180, 225)

def get_random_t():
    """t value: even number between 16 and 24"""
    return random.choice([16, 18, 20, 22, 24])

# Generate data
data = []
for i in range(num_path):
    path_id = start_id + i
    
    # Generate random endpoints and midpoints within the allowed ranges
    q1_end = get_random_j1()
    q2_end = get_random_j2()
    q3_end = get_random_j3()
    
    q1_mid = get_random_j1()
    q2_mid = get_random_j2()
    q3_mid = get_random_j3()
    
    t_total = get_random_t()
    
    data.append({
        'path_id': path_id,
        'q1_end': q1_end,
        'q2_end': q2_end,
        'q3_end': q3_end,
        'q1_mid': q1_mid,
        'q2_mid': q2_mid,
        'q3_mid': q3_mid,
        't_total': t_total
    })

# Write to CSV
csv_filename = 'endpoint_configs.csv'
with open(csv_filename, 'w', newline='') as csvfile:
    fieldnames = ['path_id', 'q1_end', 'q2_end', 'q3_end', 'q1_mid', 'q2_mid', 'q3_mid', 't_total']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    
    writer.writeheader()
    writer.writerows(data)

print(f"✓ Generated {num_path} endpoint configurations")
print(f"✓ Path IDs: {start_id} to {start_id + num_path - 1}")
print(f"✓ Output saved to: {csv_filename}")
# print(f"\nFirst 5 entries:")
# for entry in data[:5]:
#     print(entry)
