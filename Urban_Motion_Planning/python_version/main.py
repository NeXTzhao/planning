import geopandas as gpd
import matplotlib.pyplot as plt

# 读取 Shapefile 文件
# shapefile_path = "/home/vtd/Downloads/CNOA/hh05_deform_20230720/ROAD.shp"
shapefile_path = "/home/vtd/Downloads/CNOA/hh05_deform_20230720/LANE_ADAS.shp"

gdf = gpd.read_file(shapefile_path)


# 存储为 JSON 文件
# output_json_path = "LANE_ADAS.json"
# gdf.to_file(output_json_path, driver="GeoJSON")
# print("GeoDataFrame saved as JSON:", output_json_path)

# 绘制地图
ax = gdf.plot()
ax.set_title('Shapefile Visualization')
plt.show()