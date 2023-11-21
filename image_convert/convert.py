from PIL import Image

def to_dec_arr_24bit(image):
    width,height = image.size
    RGB_array = []
    for y in range(height):
        for x in range(width):
            R,G,B = image.getpixel((x,y))
            RGB_array.append(str((R<<16)+(G<<8)+B))

    return RGB_array

def to_dec_arr_32bit(image):
    width,height = image.size
    RGB_array = []
    i = 0
    bin32_array = []
    for y in range(height):
        for x in range(width):
            R,G,B = image.getpixel((x,y))
            RGB_array.append(R)
            RGB_array.append(B)
            RGB_array.append(G)
            if len(RGB_array) >= i+4:
                a,b,c,d = RGB_array[i:i+4]
                bin32_array.append(str(a + (b<<8) + (c<<16) + (d<<24)))
                i += 4

    return bin32_array

def to_image(pixel_array,size,format="RGB"):
    width,height = size
    if format == "L":
        pass
        #width,height = (width-2,height-2)
    image = Image.new(format,size)
    for y in range(height):
        for x in range(width):
            pixel = int(pixel_array[y*width+x])
            if format == "RGB":
                R,G,B = pixel>>16,(pixel>>8)&0xFF,pixel&0xFF
                image.putpixel((x,y),(R,G,B))
            elif format == "L":
                image.putpixel((x,y),pixel)

    return image

def to_image_32bit(pixel_array,size,format="RGB"):
    width,height = size
    image = Image.new(format,size)
    for y in range(height):
        for x in range(width):
            pixel = int(pixel_array[y*width+x])
            if format == "RGB":
                R,G,B = pixel>>16,(pixel>>8)&0xFF,pixel&0xFF
                image.putpixel((x,y),(R,G,B))
            elif format == "L":
                image.putpixel((x,y),pixel)

    return image

img = Image.open("/home/jonas/embedded/group_6_assignments_ros/image_convert/test_image.jpg")
img = img.convert("RGB")
width,height = img.size
img = img.resize((width,height))
print(img.size)


if False:
    RGB_array = to_dec_arr_24bit(img)
    with open("RGB_array.txt", 'w') as f:
        f.write(','.join(RGB_array))

if False:
    size = img.size
    with open("L_sobel.txt", 'r') as f:
        pixel_array = f.read().split(',')
    img = to_image(pixel_array,size,'L') # L == grayscale
    img.show()


if True:
    bin32_arr = to_dec_arr_32bit(img)
    with open("RGB_array.txt", 'w') as f:
        f.write('\n'.join(bin32_arr))

if False:
    size = img.size
    with open("L_sobel.txt", 'r') as f:
        pixel_array = f.read().split('\n')
    img = to_image_32bit(pixel_array,size,'L') # L == grayscale
    img.show()
    img.save("test_image_2_grayscale_inverted.jpg")