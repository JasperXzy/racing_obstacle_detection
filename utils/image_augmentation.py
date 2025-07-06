import torchvision.transforms as transforms
from PIL import Image
import os


def augment_images(input_folder, output_folder):
    """
    Apply mirroring and color augmentations to all images in the input folder and save them to the output folder.
    
    :param input_folder: Path to the folder containing input images.
    :param output_folder: Path to the folder where augmented images will be saved.
    """
    transform = transforms.Compose([
        transforms.RandomHorizontalFlip(p=1.0),  
        transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
        transforms.ToTensor()
    ])
    
    # Ensure output folder exists
    os.makedirs(output_folder, exist_ok=True)
    
    # Process each image in the input folder
    for filename in os.listdir(input_folder):
        if filename.lower().endswith(('.jpg', '.png')):
            img_path = os.path.join(input_folder, filename)
            image = Image.open(img_path).convert('RGB')
            augmented_image = transform(image)

            # Save the augmented image
            output_path = os.path.join(output_folder, filename)
            
            # Convert tensor back to PIL Image for saving
            augmented_image_pil = transforms.ToPILImage()(augmented_image)
            augmented_image_pil.save(output_path)
            print(f"Saved augmented image: {output_path}")


if __name__ == "__main__":
    input_folder = "../dataset/raw_images"  
    output_folder = "../dataset/obstacle/images"  
    augment_images(input_folder, output_folder)
    print("Image augmentation complete.")
