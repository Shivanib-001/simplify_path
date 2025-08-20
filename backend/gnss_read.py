import os
import json
class GNSSData:
    def __init__(self, filepath):
        """
        :param filepath - path to data file
        """
        self.filepath = filepath
        self._last_line = None

    def last_data(self):
        if not os.path.exists(self.filepath):
            print("The specified path does not exist")
            return None

        try:
            with open(self.filepath, 'r') as file:
                data=json.load(file)
                
            #print(data)
            return {"latitude": data["Latitude"], "longitude": data["Longitude"]}

        except Exception as e:
            pass
            #print(f"Error reading GPS data: {e}")

        return None

