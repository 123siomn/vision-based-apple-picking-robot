#include <astra/capi/astra.h>
#include <astra_core/capi/astra_types.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define MAX_SAMPLES 4096
#define DEFAULT_FRAME_TIMEOUT_MS 1000

typedef struct
{
    int cx;
    int cy;
    int width;
    int height;
    int valid;
    int z;
} depth_result_t;

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

/*
 * 计算一组深度样本的中位数。
 * 这里会原地排序 samples，调用方不要再依赖原始顺序。
 */
static int median_int16(int16_t *samples, int count)
{
    if (count <= 0)
    {
        return 0;
    }

    qsort(samples, count, sizeof(int16_t), cmp_int16);
    return samples[count / 2];
}

/*
 * 从单帧 depth frame 中读取目标点附近 ROI 的有效深度，并返回 ROI 中位数。
 * z=0 通常表示 Astra 没有测到有效深度，因此会被过滤掉。
 */
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

    if (width <= 0 || height <= 0 || depthLength == 0)
    {
        *out_width = 0;
        *out_height = 0;
        *out_valid = 0;
        return -1;
    }

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

    return median_int16(samples, count);
}

/*
 * 解析命令行参数并做基本限幅。
 * 第一版只接受整数像素坐标，后续检测框中心点来自 Python 的 HSV/YOLO 结果。
 */
static void parse_args(
    char *argv[],
    int *cx,
    int *cy,
    int *radius,
    int *frames_to_read
)
{
    *cx = atoi(argv[1]);
    *cy = atoi(argv[2]);
    *radius = atoi(argv[3]);
    *frames_to_read = atoi(argv[4]);

    if (*radius < 0) *radius = 0;
    if (*radius > 10) *radius = 10;
    if (*frames_to_read < 1) *frames_to_read = 1;
    if (*frames_to_read > 200) *frames_to_read = 200;
}

/*
 * 输出错误并统一返回非零状态码，方便 Python 判断失败。
 * 注意：Astra SDK 自身 warning 可能输出到 stdout/stderr，Python 端仍以 OK 行为准。
 */
static int print_error(const char *reason, int cx, int cy)
{
    printf("ERR reason=%s cx=%d cy=%d z=0 valid=0\n", reason, cx, cy);
    return 2;
}

/*
 * 清理 Astra 资源。
 * 每个标志位表示对应资源是否已经创建成功，避免异常路径重复释放。
 */
static void cleanup_astra(
    astra_reader_t *reader,
    astra_streamsetconnection_t *sensor,
    int reader_created,
    int sensor_opened,
    int astra_initialized
)
{
    if (reader_created)
    {
        astra_reader_destroy(reader);
    }
    if (sensor_opened)
    {
        astra_streamset_close(sensor);
    }
    if (astra_initialized)
    {
        astra_terminate();
    }
}

int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        fprintf(stderr, "Usage: %s cx cy radius frames\n", argv[0]);
        fprintf(stderr, "Example: %s 320 240 3 30\n", argv[0]);
        return 1;
    }

    int cx = 0;
    int cy = 0;
    int radius = 0;
    int frames_to_read = 0;

    parse_args(argv, &cx, &cy, &radius, &frames_to_read);

    /*
     * 下面几步保持 Astra 官方 DepthReaderPoll 示例的调用方式。
     * 部分 Astra SDK 版本里 open/create/get/start 这类接口是 void 返回值，
     * 因此这里不强行按返回码判断；真正取帧时再通过 astra_reader_open_frame()
     * 的返回值判断设备和数据流是否可用。
     */
    astra_initialize();
    int astra_initialized = 1;

    astra_streamsetconnection_t sensor;
    int sensor_opened = 0;
    astra_streamset_open("device/default", &sensor);
    sensor_opened = 1;

    astra_reader_t reader;
    int reader_created = 0;
    astra_reader_create(sensor, &reader);
    reader_created = 1;

    astra_depthstream_t depthStream;
    astra_reader_get_depthstream(reader, &depthStream);

    float hFov = 0.0f;
    float vFov = 0.0f;

    astra_depthstream_get_hfov(depthStream, &hFov);
    astra_depthstream_get_vfov(depthStream, &vFov);

    if (hFov <= 0.0f || vFov <= 0.0f)
    {
        cleanup_astra(&reader, &sensor, reader_created, sensor_opened, astra_initialized);
        return print_error("INVALID_FOV", cx, cy);
    }

    astra_stream_start(depthStream);

    depth_result_t frame_results[200];
    int frame_result_count = 0;
    int16_t frame_depth_samples[200];

    astra_frame_index_t lastFrameIndex = -1;

    for (int i = 0; i < frames_to_read; i++)
    {
        astra_update();

        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(reader, DEFAULT_FRAME_TIMEOUT_MS, &frame);

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
            int clamped_cx = clamp_int(cx, 0, width - 1);
            int clamped_cy = clamp_int(cy, 0, height - 1);

            frame_results[frame_result_count].cx = clamped_cx;
            frame_results[frame_result_count].cy = clamped_cy;
            frame_results[frame_result_count].width = width;
            frame_results[frame_result_count].height = height;
            frame_results[frame_result_count].valid = valid;
            frame_results[frame_result_count].z = z;
            frame_depth_samples[frame_result_count] = (int16_t)z;
            frame_result_count++;
        }

        astra_reader_close_frame(&frame);
    }

    cleanup_astra(&reader, &sensor, reader_created, sensor_opened, astra_initialized);

    if (frame_result_count <= 0)
    {
        return print_error("NO_VALID_DEPTH", cx, cy);
    }

    /*
     * 多帧稳定策略：
     * 1. 每一帧先在 ROI 内取一次中位数；
     * 2. 再对多帧 ROI 中位数取中位数，避免最后一帧偶然抖动覆盖结果。
     */
    int final_z = median_int16(frame_depth_samples, frame_result_count);
    depth_result_t final_result = frame_results[0];
    int best_diff = abs(frame_results[0].z - final_z);

    for (int i = 1; i < frame_result_count; i++)
    {
        int diff = abs(frame_results[i].z - final_z);
        if (diff < best_diff)
        {
            best_diff = diff;
            final_result = frame_results[i];
        }
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
    double fx = final_result.width / (2.0 * tan(hFov / 2.0));
    double fy = final_result.height / (2.0 * tan(vFov / 2.0));

    if (fx <= 0.0 || fy <= 0.0)
    {
        return print_error("INVALID_INTRINSIC", final_result.cx, final_result.cy);
    }

    double x_mm = (final_result.cx - final_result.width / 2.0) * final_z / fx;
    double y_mm = (final_result.cy - final_result.height / 2.0) * final_z / fy;
    double z_mm = final_z;

    printf(
        "OK cx=%d cy=%d width=%d height=%d x=%.1f y=%.1f z=%.1f valid=%d hFov=%.6f vFov=%.6f\n",
        final_result.cx,
        final_result.cy,
        final_result.width,
        final_result.height,
        x_mm,
        y_mm,
        z_mm,
        final_result.valid,
        hFov,
        vFov
    );

    return 0;
}
