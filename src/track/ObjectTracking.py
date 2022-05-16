from tracker import Tracker


extention = ".jpg"
imagelist = traverseFolder(IMAGE_DIR, extention);  # traverse( IMAGE_DIR , extention);#
imagelist = sorted(imagelist)
det_list = [1, 2, 3, 4, 6, 8]
counter = 0
tracker = Tracker(60, 30, 300, 100)
def ObjTracking():
    for frame in range(len(results)):
        bb_number = 0
        centers = []
        r = results[frame]
        out_boxes = r['rois']
        out_scores = r['scores']
        out_classes = r['class_ids']

        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = class_names[c]
            box = out_boxes[i]
            score = out_scores[i]
            bb_number = len(out_boxes)
            if c in det_list:
                top, left, bottom, right = box
                center = (int((left + right) // 2), int((top + bottom) // 2))
                b = np.array([[(left + right) // 2], [(top + bottom) // 2]])
                centers.append(b)

        image = skimage.io.imread(imagelist[frame])
        # visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores'])
        if counter % 40 == 0:
            print('processed image', counter)
        counter += 1

        if (len(centers) > 0):
            # print(len(tracker.tracks))
            tracker.Update(centers)


