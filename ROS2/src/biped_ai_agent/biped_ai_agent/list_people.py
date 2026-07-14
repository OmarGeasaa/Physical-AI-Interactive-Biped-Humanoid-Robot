import os
import argparse

def list_people(faces_dir="known_faces"):
    print("\n" + "="*50)
    print("👤 People in Face Database")
    print("="*50)
    
    if not os.path.exists(faces_dir):
        print(f"❌ Directory '{faces_dir}' not found!")
        return
    
    person_folders = [f for f in os.listdir(faces_dir) 
                     if os.path.isdir(os.path.join(faces_dir, f))]
    
    if not person_folders:
        print(f"❌ No person subfolders found in '{faces_dir}'")
        return
    
    total_images = 0
    
    for person in sorted(person_folders):
        person_dir = os.path.join(faces_dir, person)
        images = [f for f in os.listdir(person_dir) 
                 if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.gif'))]
        total_images += len(images)
        
        print(f"\n👤 {person}")
        print(f"   📸 {len(images)} photos")
        if images:
            print(f"   📋 Files: {', '.join(images[:3])}")
            if len(images) > 3:
                print(f"   ... and {len(images) - 3} more")
    
    print("\n" + "="*50)
    print(f"📊 Total: {len(person_folders)} people, {total_images} photos")
    print("="*50)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='List people in face database')
    parser.add_argument('--faces-dir', type=str, default='known_faces',
                       help='Path to folder with person subfolders')
    args = parser.parse_args()
    list_people(args.faces_dir)