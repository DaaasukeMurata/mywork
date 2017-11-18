import matplotlib.pyplot as plt
from PIL import Image

# plt.style.use('ggplot')

image = Image.open('./sample1.jpg')

# ok
# image.show()

plt.figure(figsize=(12, 8))
plt.imshow(image)
plt.show()