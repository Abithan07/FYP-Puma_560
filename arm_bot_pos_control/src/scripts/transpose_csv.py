import csv
import sys

def transpose_csv(input_file, output_file):
    """
    Transpose CSV file from row-wise to column-wise format.
    
    Input format (row-wise):
    tau1,value1,value2,value3,...
    tau2,value1,value2,value3,...
    tau3,value1,value2,value3,...
    
    Output format (column-wise):
    tau1,tau2,tau3
    value1,value1,value1
    value2,value2,value2
    value3,value3,value3
    ...
    """
    try:
        # Read the input CSV file
        with open(input_file, 'r') as infile:
            reader = csv.reader(infile)
            rows = list(reader)
        
        if len(rows) == 0:
            print("Error: Input file is empty")
            return False
        
        # Extract row names (tau1, tau2, tau3) from first column
        row_names = [row[0] for row in rows]
        
        # Extract data values (skip first column which has names)
        data_values = [row[1:] for row in rows]
        
        # Check if all rows have the same number of columns
        num_cols = len(data_values[0])
        if not all(len(row) == num_cols for row in data_values):
            print("Warning: Not all rows have the same number of values")
        
        # Transpose the data
        transposed_data = []
        for col_idx in range(num_cols):
            transposed_row = [data_values[row_idx][col_idx] if col_idx < len(data_values[row_idx]) else '' 
                            for row_idx in range(len(data_values))]
            transposed_data.append(transposed_row)
        
        # Write to output CSV file
        with open(output_file, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            
            # Write header with row names (tau1, tau2, tau3)
            writer.writerow(row_names)
            
            # Write transposed data
            writer.writerows(transposed_data)
        
        print(f"Successfully transposed {len(rows)} rows with {num_cols} values each")
        print(f"Output saved to: {output_file}")
        print(f"New format: {len(transposed_data)} rows × {len(row_names)} columns")
        return True
    
    except FileNotFoundError:
        print(f"Error: Input file '{input_file}' not found")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python transpose_csv.py <input_file> [output_file]")
        print("\nExample:")
        print("  python transpose_csv.py torque_data.csv torque_values.csv")
        print("\nIf output_file is not specified, it will use 'torque_values.csv'")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else 'torque_values.csv'
    
    transpose_csv(input_file, output_file)

if __name__ == '__main__':
    main()