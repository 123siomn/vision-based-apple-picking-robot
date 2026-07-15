#include <astra/capi/astra.h>
#include <astra_core/capi/astra_types.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define MAX_SAMPLES 4096

static int cmp_int16(const void *a, const void *b)
{
    int va = *(const int16_t *)a;
    int vb = *(const int16_t *)b;
    return va - vb;
}

static int clamp_int(int v, int min_v, int max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static int get_roi_median_depth(
    astra_depthframe_t depthFrame,
    int cx,
    int cy,
    int radius,
    int *out_width,
    int *out_height,
    int *out_valid
)
{
    astra_image_metadata_t metadata;

    int16_t *depthData = NULL;
    uint32_t depthLength = 0;

    astra_depthframe_get_data_byte_length(depthFrame, &depthLength);
    astra_depthframe_get_metadata(depthFrame, &metadata);

    int width = metadata.width;
    int height = metadata.height;

    *out_width = width;
    *out_height = height;

    depthData = (int16_t *)malloc(depthLength);
    if (depthData == NULL)
    {
        return -1;
    }

    astra_depthframe_copy_data(depthFrame, depthData);

    cx = clamp_int(cx, 0, width - 1);
    cy = clamp_int(cy, 0, height - 1);

    int16_t samples[MAX_SAMPLES];
    int count = 0;

    for (int dy = -radius; dy <= radius; dy++)
    {
        for (int dx = -radius; dx <= radius; dx++)
        {
            int x = cx + dx;
            int y = cy + dy;

            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }

            int16_t z = depthData[y * width + x];

            /*
             * z=0 通常表示无效深度点。
             * Astra 输出单位通常是 mm。
             */
            if (z > 0)
            {
                if (count < MAX_SAMPLES)
                {
                    samples[count++] = z;
                }
            }
        }
    }

    free(depthData);

    *out_valid = count;

    if (count == 0)
    {
        return 0;
    }

    qsort(samples, count, sizeof(int16_t), cmp_int16);

    return samples[count / 2];
}

int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        fprintf(stderr, "Usage: %s cx cy radius frames\n", argv[0]);
        fprintf(stderr, "Example: %s 320 240 3 30\n", argv[0]);
        return 1;
    }

    int cx = atoi(argv[1]);
    int cy = atoi(argv[2]);
    int radius = atoi(argv[3]);
    int frames_to_read = atoi(argv[4]);

    if (radius < 0) radius = 0;
    if (radius > 10) radius = 10;
    if (frames_to_read < 1) frames_to_read = 1;
    if (frames_to_read > 200) frames_to_read = 200;

    astra_initialize();

    astra_streamsetconnection_t sensor;
    astra_streamset_open("device/default", &sensor);

    astra_reader_t reader;
    astra_reader_create(sensor, &reader);

    astra_depthstream_t depthStream;
    astra_reader_get_depthstream(reader, &depthStream);

    float hFov = 0.0f;
    float vFov = 0.0f;

    astra_depthstream_get_hfov(depthStream, &hFov);
    astra_depthstream_get_vfov(depthStream, &vFov);

    astra_stream_start(depthStream);

    int final_z = 0;
    int final_valid = 0;
    int final_width = 0;
    int final_height = 0;
    int got_valid_frame = 0;

    astra_frame_index_t lastFrameIndex = -1;

    for (int i = 0; i < frames_to_read; i++)
    {
        astra_update();

        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(reader, 1000, &frame);

        if (rc != ASTRA_STATUS_SUCCESS)
        {
            continue;
        }

        astra_depthframe_t depthFrame;
        astra_frame_get_depthframe(frame, &depthFrame);

        astra_frame_index_t newFrameIndex;
        astra_depthframe_get_frameindex(depthFrame, &newFrameIndex);

        if (newFrameIndex == lastFrameIndex)
        {
            astra_reader_close_frame(&frame);
            continue;
        }

        lastFrameIndex = newFrameIndex;

        int width = 0;
        int height = 0;
        int valid = 0;

        int z = get_roi_median_depth(
            depthFrame,
            cx,
            cy,
            radius,
            &width,
            &height,
            &valid
        );

        if (z > 0 && valid > 0)
        {
            final_z = z;
            final_valid = valid;
            final_width = width;
            final_height = height;
            got_valid_frame = 1;
        }

        astra_reader_close_frame(&frame);
    }

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);
    astra_terminate();

    if (!got_valid_frame)
    {
        printf("ERR cx=%d cy=%d z=0 valid=0\n", cx, cy);
        return 2;
    }

    /*
     * 用视场角粗略把像素坐标换算成相机坐标。
     *
     * 坐标含义：
     * X：图像右侧为正
     * Y：图像下方为正
     * Z：相机前方距离
     *
     * 单位：mm
     */
    double fx = final_width / (2.0 * tan(hFov / 2.0));
    double fy = final_height / (2.0 * tan(vFov / 2.0));

    double x_mm = (cx - final_width / 2.0) * final_z / fx;
    double y_mm = (cy - final_height / 2.0) * final_z / fy;
    double z_mm = final_z;

    printf(
        "OK cx=%d cy=%d width=%d height=%d x=%.1f y=%.1f z=%.1f valid=%d hFov=%.6f vFov=%.6f\n",
        cx,
        cy,
        final_width,
        final_height,
        x_mm,
        y_mm,
        z_mm,
        final_valid,
        hFov,
        vFov
    );

    return 0;
}
