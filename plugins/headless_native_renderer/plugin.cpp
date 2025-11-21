#include <array>
#include <cassert>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <vector>
#include <vulkan/vulkan_core.h>

#define VMA_IMPLEMENTATION
#include "illixr/global_module_defs.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/pose_prediction.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"
#include "illixr/vk_util/headless_sink.hpp"
#include "illixr/vk_util/render_pass.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "illixr/gl_util/lib/tiny_obj_loader.h"

using namespace ILLIXR;

class native_renderer : public threadloop {
public:
    native_renderer(const std::string& name_, phonebook* pb)
        : threadloop{name_, pb}
        , sb{pb->lookup_impl<switchboard>()}
        , pp{pb->lookup_impl<pose_prediction>()}
        , hs{pb->lookup_impl<headless_sink>()}
        , tw{pb->lookup_impl<timewarp>()}
        , src{pb->lookup_impl<app>()}
        , _m_clock{pb->lookup_impl<RelativeClock>()}
        , last_fps_update{std::chrono::duration<long, std::nano>{0}} {
        spdlogger(std::getenv("NATIVE_RENDERER_LOG_LEVEL"));
    }

    /**
     * @brief Sets up the thread for the plugin.
     *
     * This function initializes depth images, offscreen targets, command buffers, sync objects,
     * application and timewarp passes, offscreen and swapchain framebuffers. Then, it initializes
     * application and timewarp with their respective passes.
     */
    void _p_thread_setup() override {
        for (auto i = 0; i < 2; i++) {
            create_depth_image(&depth_images[i], &depth_image_allocations[i], &depth_image_views[i]);
        }
        for (auto i = 0; i < 2; i++) {
            create_offscreen_target(&offscreen_images[i], &offscreen_image_allocations[i], &offscreen_image_views[i],
                                    &offscreen_framebuffers[i]);
        }
        command_pool            = vulkan_utils::create_command_pool(hs->vk_device, hs->graphics_queue_family);
        app_command_buffer      = vulkan_utils::create_command_buffer(hs->vk_device, command_pool);
        timewarp_command_buffer = vulkan_utils::create_command_buffer(hs->vk_device, command_pool);
        create_sync_objects();
        create_app_pass();
        create_timewarp_pass();
        create_sync_objects();
        create_offscreen_framebuffers();
        create_framebuffer();
        src->setup(app_pass, 0);
        tw->setup(timewarp_pass, 0, {std::vector{offscreen_image_views[0]}, std::vector{offscreen_image_views[1]}}, true);
    }

    /**
     * @brief Executes one iteration of the plugin's main loop.
     *
     * This function handles window events, acquires the next image from the swapchain, updates uniforms,
     * records command buffers, submits commands to the graphics queue, and presents the rendered image.
     * It also handles swapchain recreation if necessary and updates the frames per second (FPS) counter.
     *
     * @throws runtime_error If any Vulkan operation fails.
     */
    void _p_one_iteration() override {

        // Wait for the previous frame to finish rendering
        VK_ASSERT_SUCCESS(vkWaitForFences(hs->vk_device, 1, &frame_fence, VK_TRUE, UINT64_MAX))

        VK_ASSERT_SUCCESS(vkResetFences(hs->vk_device, 1, &frame_fence))

        // Get the current fast pose and update the uniforms
        auto fast_pose = pp->get_fast_pose();
        src->update_uniforms(fast_pose.pose);

        // Record the command buffer
        VK_ASSERT_SUCCESS(vkResetCommandBuffer(app_command_buffer, 0))
        record_command_buffer();

        // Submit the command buffer to the graphics queue
        const uint64_t ignored     = 0;
        const uint64_t fired_value = timeline_semaphore_value + 1;

        timeline_semaphore_value += 1;
        VkTimelineSemaphoreSubmitInfo timeline_submit_info{
            VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO, // sType
            nullptr,                                          // pNext
            0,                                                // waitSemaphoreValueCount
            &ignored,                                         // pWaitSemaphoreValues
            1,                                                // signalSemaphoreValueCount
            &fired_value                                      // pSignalSemaphoreValues
        };

        VkPipelineStageFlags wait_stages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
        VkSubmitInfo         submit_info{
            VK_STRUCTURE_TYPE_SUBMIT_INFO, // sType
            &timeline_submit_info,         // pNext
            0,                             // waitSemaphoreCount
            nullptr,                    // pWaitSemaphores
            nullptr,                   // pWaitDstStageMask
            1,                             // commandBufferCount
            &app_command_buffer,           // pCommandBuffers
            1,                             // signalSemaphoreCount
            &app_render_finished_semaphore // pSignalSemaphores
        };

        VK_ASSERT_SUCCESS(vkQueueSubmit(hs->graphics_queue, 1, &submit_info, nullptr))

        // Wait for the application to finish rendering
        VkSemaphoreWaitInfo wait_info{
            VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO, // sType
            nullptr,                               // pNext
            0,                                     // flags
            1,                                     // semaphoreCount
            &app_render_finished_semaphore,        // pSemaphores
            &fired_value                           // pValues
        };
        VK_ASSERT_SUCCESS(vkWaitSemaphores(hs->vk_device, &wait_info, UINT64_MAX))

        // TODO: for DRM, get vsync estimate
        std::this_thread::sleep_for(display_params::period / 6.0 * 5);

        // Update the timewarp uniforms and submit the timewarp command buffer to the graphics queue
        tw->update_uniforms(fast_pose.pose);
        VkSubmitInfo timewarp_submit_info{
            VK_STRUCTURE_TYPE_SUBMIT_INFO,      // sType
            nullptr,                            // pNext
            0,                                  // waitSemaphoreCount
            nullptr,                            // pWaitSemaphores
            nullptr,                            // pWaitDstStageMask
            1,                                  // commandBufferCount
            &timewarp_command_buffer,           // pCommandBuffers
            0,                                  // signalSemaphoreCount
            nullptr                              // pSignalSemaphores
        };

        VK_ASSERT_SUCCESS(vkQueueSubmit(hs->graphics_queue, 1, &timewarp_submit_info, frame_fence))
        // std::cout << "MTP: " << duration2double<std::milli>(_m_clock->now() - fast_pose.pose.sensor_time) << std::endl;

        if (frame_count % 200 == 0) {

            // wait sfor frame to finish rendering
            vkWaitForFences(hs->vk_device, 1, &frame_fence, VK_TRUE, UINT64_MAX);

            // create image in host memory 
            VkImage dstImage;

            VkImageCreateInfo imageInfo{};
            imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.extent.width = hs->extent.width;
            imageInfo.extent.height = hs->extent.height;
            imageInfo.extent.depth = 1;
            imageInfo.mipLevels = 1;
            imageInfo.arrayLayers = 1;
            imageInfo.format = hs->image_format;
            imageInfo.tiling = VK_IMAGE_TILING_LINEAR;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;

            if (vkCreateImage(hs->vk_device, &imageInfo, nullptr, &dstImage) != VK_SUCCESS) {
                throw std::runtime_error("failed to create destination image!");
            }
            
            VkDeviceMemory dstImageMemory;

            VkMemoryRequirements memRequirements;
            vkGetImageMemoryRequirements(hs->vk_device, dstImage, &memRequirements);
            
            VkMemoryAllocateInfo allocInfo{};
            allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            allocInfo.allocationSize = memRequirements.size;
            allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits,  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            
            if (vkAllocateMemory(hs->vk_device, &allocInfo, nullptr, &dstImageMemory) != VK_SUCCESS) {
                throw std::runtime_error("failed to allocate image memory!");
            }
            vkBindImageMemory(hs->vk_device, dstImage, dstImageMemory, 0);

            // transition dstImage to optimal layout for recieving the image
            transitionImageLayout(dstImage, hs->image_format, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

            // copy image
            VkCommandBuffer commandBuffer = beginSingleTimeCommands();

            VkImageCopy imageCopyRegion{};
            imageCopyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            imageCopyRegion.srcSubresource.layerCount = 1;
            imageCopyRegion.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            imageCopyRegion.dstSubresource.layerCount = 1;
            imageCopyRegion.extent.width = hs->extent.width;
            imageCopyRegion.extent.height = hs->extent.height;
            imageCopyRegion.extent.depth = 1;
            
            vkCmdCopyImage(
                           commandBuffer,
                           hs->image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           dstImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           1,
                           &imageCopyRegion);
            
            // submit command buffer
            endSingleTimeCommands(commandBuffer, copy_frame_fence);

            // wait for copy to complete
            vkWaitForFences(hs->vk_device, 1, &copy_frame_fence, VK_TRUE, UINT64_MAX);
            
            // transition image to general layout to write to file later
            transitionImageLayout(dstImage, VK_FORMAT_B8G8R8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL);

            // Get layout of the image (including row pitch)
            VkImageSubresource subResource{};
            subResource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            VkSubresourceLayout subResourceLayout;
            vkGetImageSubresourceLayout(hs->vk_device, dstImage, &subResource, &subResourceLayout);
            
            // Map image memory to a pointer so we can start copying from it
            const char* imagedata;
            vkMapMemory(hs->vk_device, dstImageMemory, 0, VK_WHOLE_SIZE, 0, (void**)&imagedata);
            imagedata += subResourceLayout.offset;

            // filename lol
            auto formatted = [](const char* format, auto... args) {
                size_t size = snprintf(nullptr, 0, format, args...) + 1;
                std::string result(size, '\0');
                snprintf(&result[0], size, format, args...);
                return result;
            };
            std::string fname = formatted("/scratch/prashanth/ILLIXR-headless-vulkan/build/saved_frames/%d.ppm", frame_count);
            const char* filename = fname.c_str();

            std::ofstream file(filename, std::ofstream::binary);
            // ppm header
            file << "P6\n" << hs->extent.width << "\n" << hs->extent.height << "\n" << 255 << "\n";

            
            for (int32_t y = 0; y < hs->extent.height; y++) {
                unsigned int *row = (unsigned int*)imagedata;
                for (int32_t x = 0; x < hs->extent.width; x++) {
                    
                    // swizzle colors because format is VK_FORMAT_B8G8R8A8_SRGB, so switch BGR to RGB
                    file.write((char*)row+2, 1);
                    file.write((char*)row+1, 1);
                    file.write((char*)row, 1);
                    row++;
                }
                imagedata += subResourceLayout.rowPitch;
            }
            file.close();


            std::cout << "saved " << fname.c_str() << std::endl;
            
            // reset fence for copy operation
            vkResetFences(hs->vk_device, 1, &copy_frame_fence);
            
            // unmap and free memory
            vkUnmapMemory(hs->vk_device, dstImageMemory);
            vkFreeMemory(hs->vk_device, dstImageMemory, nullptr);
            vkDestroyImage(hs->vk_device, dstImage, nullptr);

        }

        frame_count++;

        // #ifndef NDEBUG
        // Print the FPS
        if (_m_clock->now() - last_fps_update > std::chrono::milliseconds(1000)) {
            // std::cout << "FPS: " << fps << std::endl;
            fps             = 0;
            last_fps_update = _m_clock->now();
        } else {
            fps++;
        }
        // #endif
    }

private:

    void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout) {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();
        
        // use image memory barrier (usually used to synchronize access to resources) to transition image layouts and queue family ownership with exclusive sharing mode
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        
        // specify image and specific part of image (not an array and no mipmapping levels so only 1 level and layer specified)
        barrier.image = image;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;
        
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = 0;
        
        VkPipelineStageFlags sourceStage, destinationStage;
        if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            
            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_GENERAL) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
            
            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        } else {
            throw std::invalid_argument("unsupported layout transition!");
        }
        
        vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
        
        endSingleTimeCommands(commandBuffer, VK_NULL_HANDLE);
    }

    VkCommandBuffer beginSingleTimeCommands() {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = command_pool;
        allocInfo.commandBufferCount = 1;
        
        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(hs->vk_device, &allocInfo, &commandBuffer);
        
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        
        vkBeginCommandBuffer(commandBuffer, &beginInfo);
        
        return commandBuffer;
    }

    void endSingleTimeCommands(VkCommandBuffer commandBuffer, VkFence fence) {
        vkEndCommandBuffer(commandBuffer);
        
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        
        vkQueueSubmit(hs->graphics_queue, 1, &submitInfo, fence);
        vkQueueWaitIdle(hs->graphics_queue);
        vkFreeCommandBuffers(hs->vk_device, command_pool, 1, &commandBuffer);
    }

    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
        // has memoryTypes and memoryHeaps
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(hs->vk_physical_device, &memProperties);
        
        // check which memory type has the properties we want
        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }
        throw std::runtime_error("failed to find suitable memory type!");
    }

    /**
     * @brief Creates framebuffer
     *
     * @throws runtime_error If framebuffer creation fails.
     */
    void create_framebuffer() {

        std::array<VkImageView, 1> attachments = {hs->image_view};

        assert(timewarp_pass != VK_NULL_HANDLE);
        VkFramebufferCreateInfo framebuffer_info{
            VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO, // sType
            nullptr,                                   // pNext
            0,                                         // flags
            timewarp_pass,                             // renderPass
            attachments.size(),                        // attachmentCount
            attachments.data(),                        // pAttachments
            hs->extent.width,                           // width
            hs->extent.height,                          // height
            1                                          // layers
        };

        VK_ASSERT_SUCCESS(vkCreateFramebuffer(hs->vk_device, &framebuffer_info, nullptr, &framebuffer))
        
    }

    /**
     * @brief Records the command buffer for a single frame.
     * 
     */
    void record_command_buffer() {
        // Begin recording app command buffer
        VkCommandBufferBeginInfo begin_info = {
            VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, // sType
            nullptr,                                     // pNext
            0,                                           // flags
            nullptr                                      // pInheritanceInfo
        };
        VK_ASSERT_SUCCESS(vkBeginCommandBuffer(app_command_buffer, &begin_info))

        for (auto eye = 0; eye < 2; eye++) {
            assert(app_pass != VK_NULL_HANDLE);
            std::array<VkClearValue, 2> clear_values = {};
            clear_values[0].color                    = {{1.0f, 1.0f, 1.0f, 1.0f}};
            clear_values[1].depthStencil             = {1.0f, 0};

            VkRenderPassBeginInfo render_pass_info{
                VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO, // sType
                nullptr,                                  // pNext
                app_pass,                                 // renderPass
                offscreen_framebuffers[eye],              // framebuffer
                {
                    {0, 0},              // offset
                    hs->extent           // extent
                },                       // renderArea
                clear_values.size(),     // clearValueCount
                clear_values.data()      // pClearValues
            };

            vkCmdBeginRenderPass(app_command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);
            // Call app service to record the command buffer
            src->record_command_buffer(app_command_buffer, eye);
            vkCmdEndRenderPass(app_command_buffer);
        }
        VK_ASSERT_SUCCESS(vkEndCommandBuffer(app_command_buffer))

        // Begin recording timewarp command buffer
        VkCommandBufferBeginInfo timewarp_begin_info = {
            VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, // sType
            nullptr,                                     // pNext
            0,                                           // flags
            nullptr                                      // pInheritanceInfo
        };
        VK_ASSERT_SUCCESS(vkBeginCommandBuffer(timewarp_command_buffer, &timewarp_begin_info)) {
            assert(timewarp_pass != VK_NULL_HANDLE);
            VkClearValue          clear_value{.color = {{0.0f, 0.0f, 0.0f, 1.0f}}};
            VkRenderPassBeginInfo render_pass_info{
                VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,      // sType
                nullptr,                                       // pNext
                timewarp_pass,                                 // renderPass
                framebuffer,                                    // framebuffer
                {
                    {0, 0},              // offset
                    hs->extent          // extent
                },                       // renderArea
                1,                       // clearValueCount
                &clear_value             // pClearValues
            };

            vkCmdBeginRenderPass(timewarp_command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);

            for (auto eye = 0; eye < 2; eye++) {
                VkViewport viewport{
                    static_cast<float>(hs->extent.width / 2. * eye),            // x
                    0.0f,                                                      // y
                    static_cast<float>(hs->extent.width),                       // width
                    static_cast<float>(hs->extent.height),                      // height
                    0.0f,                                                      // minDepth
                    1.0f                                                       // maxDepth
                };
                vkCmdSetViewport(timewarp_command_buffer, 0, 1, &viewport);

                VkRect2D scissor{
                    {0, 0},              // offset
                    hs->extent          // extent
                };
                vkCmdSetScissor(timewarp_command_buffer, 0, 1, &scissor);

                // Call timewarp service to record the command buffer
                tw->record_command_buffer(timewarp_command_buffer, 0, eye == 0);
            }
            vkCmdEndRenderPass(timewarp_command_buffer);
        }
        VK_ASSERT_SUCCESS(vkEndCommandBuffer(timewarp_command_buffer))
    }

    /**
     * @brief Creates synchronization objects for the application.
     *
     * This function creates a timeline semaphore for the application render finished signal,
     * a binary semaphore for the image available signal, a binary semaphore for the timewarp render finished signal,
     * and a fence for frame synchronization.
     *
     * @throws runtime_error If any Vulkan operation fails.
     */
    void create_sync_objects() {
        VkSemaphoreTypeCreateInfo timeline_semaphore_info{
            VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO, // sType
            nullptr,                                      // pNext
            VK_SEMAPHORE_TYPE_TIMELINE,                   // semaphoreType
            0                                             // initialValue
        };

        VkSemaphoreCreateInfo create_info{
            VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO, // sType
            &timeline_semaphore_info,                // pNext
            0                                        // flags
        };

        vkCreateSemaphore(hs->vk_device, &create_info, nullptr, &app_render_finished_semaphore);

        VkSemaphoreCreateInfo semaphore_info{
            VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO, // sType
            nullptr,                                 // pNext
            0                                        // flags
        };
        VkFenceCreateInfo fence_info{
            VK_STRUCTURE_TYPE_FENCE_CREATE_INFO, // sType
            nullptr,                             // pNext
            VK_FENCE_CREATE_SIGNALED_BIT         // flags
        };

        VK_ASSERT_SUCCESS(vkCreateSemaphore(hs->vk_device, &semaphore_info, nullptr, &image_available_semaphore))
        VK_ASSERT_SUCCESS(vkCreateSemaphore(hs->vk_device, &semaphore_info, nullptr, &timewarp_render_finished_semaphore))
        VK_ASSERT_SUCCESS(vkCreateFence(hs->vk_device, &fence_info, nullptr, &frame_fence))

        VkFenceCreateInfo copy_fence_info{
            VK_STRUCTURE_TYPE_FENCE_CREATE_INFO, // sType
            nullptr,                             // pNext
        };
        VK_ASSERT_SUCCESS(vkCreateFence(hs->vk_device, &copy_fence_info, nullptr, &copy_frame_fence))
    }

    /**
     * @brief Creates a depth image for the application.
     * @param depth_image Pointer to the depth image handle.
     * @param depth_image_allocation Pointer to the depth image memory allocation handle.
     * @param depth_image_view Pointer to the depth image view handle.
     */
    void create_depth_image(VkImage* depth_image, VmaAllocation* depth_image_allocation, VkImageView* depth_image_view) {
        VkImageCreateInfo image_info{
            VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO, // sType
            nullptr,                             // pNext
            0,                                   // flags
            VK_IMAGE_TYPE_2D,                    // imageType
            VK_FORMAT_D32_SFLOAT,                // format
            {
                display_params::width_pixels,                                         // width
                display_params::height_pixels,                                        // height
                1                                                                     // depth
            },                                                                        // extent
            1,                                                                        // mipLevels
            1,                                                                        // arrayLayers
            VK_SAMPLE_COUNT_1_BIT,                                                    // samples
            VK_IMAGE_TILING_OPTIMAL,                                                  // tiling
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, // usage
            {},                                                                       // sharingMode
            0,                                                                        // queueFamilyIndexCount
            nullptr,                                                                  // pQueueFamilyIndices
            {}                                                                        // initialLayout
        };

        VmaAllocationCreateInfo alloc_info{};
        alloc_info.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        alloc_info.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;

        VK_ASSERT_SUCCESS(
            vmaCreateImage(hs->vma_allocator, &image_info, &alloc_info, depth_image, depth_image_allocation, nullptr))

        VkImageViewCreateInfo view_info{
            VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO, // sType
            nullptr,                                  // pNext
            0,                                        // flags
            *depth_image,                             // image
            VK_IMAGE_VIEW_TYPE_2D,                    // viewType
            VK_FORMAT_D32_SFLOAT,                     // format
            {},                                       // components
            {
                VK_IMAGE_ASPECT_DEPTH_BIT, // aspectMask
                0,                         // baseMipLevel
                1,                         // levelCount
                0,                         // baseArrayLayer
                1                          // layerCount
            }                              // subresourceRange
        };

        VK_ASSERT_SUCCESS(vkCreateImageView(hs->vk_device, &view_info, nullptr, depth_image_view))
    }

    /**
     * @brief Creates an offscreen target for the application to render to.
     * @param offscreen_image Pointer to the offscreen image handle.
     * @param offscreen_image_allocation Pointer to the offscreen image memory allocation handle.
     * @param offscreen_image_view Pointer to the offscreen image view handle.
     * @param offscreen_framebuffer Pointer to the offscreen framebuffer handle.
     */
    void create_offscreen_target(VkImage* offscreen_image, VmaAllocation* offscreen_image_allocation,
                                 VkImageView* offscreen_image_view, [[maybe_unused]] VkFramebuffer* offscreen_framebuffer) {
        VkImageCreateInfo image_info{
            VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO, // sType
            nullptr,                             // pNext
            0,                                   // flags
            VK_IMAGE_TYPE_2D,                    // imageType
            VK_FORMAT_B8G8R8A8_UNORM,            // format
            {
                display_params::width_pixels,                                 // width
                display_params::height_pixels,                                // height
                1                                                             // depth
            },                                                                // extent
            1,                                                                // mipLevels
            1,                                                                // arrayLayers
            VK_SAMPLE_COUNT_1_BIT,                                            // samples
            VK_IMAGE_TILING_OPTIMAL,                                          // tiling
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, // usage
            {},                                                               // sharingMode
            0,                                                                // queueFamilyIndexCount
            nullptr,                                                          // pQueueFamilyIndices
            {}                                                                // initialLayout
        };

        VmaAllocationCreateInfo alloc_info = {};
        alloc_info.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        alloc_info.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;

        VK_ASSERT_SUCCESS(
            vmaCreateImage(hs->vma_allocator, &image_info, &alloc_info, offscreen_image, offscreen_image_allocation, nullptr))

        VkImageViewCreateInfo view_info{
            VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO, // sType
            nullptr,                                  // pNext
            0,                                        // flags
            *offscreen_image,                         // image
            VK_IMAGE_VIEW_TYPE_2D,                    // viewType
            VK_FORMAT_B8G8R8A8_UNORM,                 // format
            {},                                       // components
            {
                VK_IMAGE_ASPECT_COLOR_BIT, // aspectMask
                0,                         // baseMipLevel
                1,                         // levelCount
                0,                         // baseArrayLayer
                1                          // layerCount
            }                              // subresourceRange
        };

        VK_ASSERT_SUCCESS(vkCreateImageView(hs->vk_device, &view_info, nullptr, offscreen_image_view))
    }

    /**
     * @brief Creates the offscreen framebuffers for the application.
     */
    void create_offscreen_framebuffers() {
        for (auto eye = 0; eye < 2; eye++) {
            std::array<VkImageView, 2> attachments = {offscreen_image_views[eye], depth_image_views[eye]};

            assert(app_pass != VK_NULL_HANDLE);
            VkFramebufferCreateInfo framebuffer_info{
                VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO, // sType
                nullptr,                                   // pNext
                0,                                         // flags
                app_pass,                                  // renderPass
                static_cast<uint32_t>(attachments.size()), // attachmentCount
                attachments.data(),                        // pAttachments
                display_params::width_pixels,              // width
                display_params::height_pixels,             // height
                1                                          // layers
            };

            VK_ASSERT_SUCCESS(vkCreateFramebuffer(hs->vk_device, &framebuffer_info, nullptr, &offscreen_framebuffers[eye]))
        }
    }

    /**
     * @brief Creates a render pass for the application.
     *
     * This function sets up the attachment descriptions for color and depth, the attachment references,
     * the subpass description, and the subpass dependencies. It then creates a render pass with these configurations.
     *
     * @throws runtime_error If render pass creation fails.
     */
    void create_app_pass() {
        std::array<VkAttachmentDescription, 2> attchmentDescriptions{
            {{
                 0,                                       // flags
                 VK_FORMAT_B8G8R8A8_UNORM,                // format
                 VK_SAMPLE_COUNT_1_BIT,                   // samples
                 VK_ATTACHMENT_LOAD_OP_CLEAR,             // loadOp
                 VK_ATTACHMENT_STORE_OP_STORE,            // storeOp
                 VK_ATTACHMENT_LOAD_OP_DONT_CARE,         // stencilLoadOp
                 VK_ATTACHMENT_STORE_OP_DONT_CARE,        // stencilStoreOp
                 VK_IMAGE_LAYOUT_UNDEFINED,               // initialLayout
                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL // finalLayout
             },
             {
                 0,                                               // flags
                 VK_FORMAT_D32_SFLOAT,                            // format
                 VK_SAMPLE_COUNT_1_BIT,                           // samples
                 VK_ATTACHMENT_LOAD_OP_CLEAR,                     // loadOp
                 VK_ATTACHMENT_STORE_OP_DONT_CARE,                // storeOp
                 VK_ATTACHMENT_LOAD_OP_DONT_CARE,                 // stencilLoadOp
                 VK_ATTACHMENT_STORE_OP_DONT_CARE,                // stencilStoreOp
                 VK_IMAGE_LAYOUT_UNDEFINED,                       // initialLayout
                 VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL // finalLayout
             }}};

        VkAttachmentReference color_attachment_ref{
            0,                                       // attachment
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL // layout
        };

        VkAttachmentReference depth_attachment_ref{
            1,                                               // attachment
            VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL // layout
        };

        VkSubpassDescription subpass = {
            0,                               // flags
            VK_PIPELINE_BIND_POINT_GRAPHICS, // pipelineBindPoint
            0,                               // inputAttachmentCount
            nullptr,                         // pInputAttachments
            1,                               // colorAttachmentCount
            &color_attachment_ref,           // pColorAttachments
            nullptr,                         // pResolveAttachments
            &depth_attachment_ref,           // pDepthStencilAttachment
            0,                               // preserveAttachmentCount
            nullptr                          // pPreserveAttachments
        };

        // Subpass dependencies for layout transitions
        std::array<VkSubpassDependency, 2> dependencies{
            {{
                 // After timewarp samples from the offscreen image, it needs to be transitioned to a color attachment
                 VK_SUBPASS_EXTERNAL,                           // srcSubpass
                 0,                                             // dstSubpass
                 VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,         // srcStageMask
                 VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, // dstStageMask
                 VK_ACCESS_SHADER_READ_BIT,                     // srcAccessMask
                 VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,          // dstAccessMask
                 VK_DEPENDENCY_BY_REGION_BIT                    // dependencyFlags
             },
             {
                 // After the app is done rendering to the offscreen image, it needs to be transitioned to a shader read
                 0,                                             // srcSubpass
                 VK_SUBPASS_EXTERNAL,                           // dstSubpass
                 VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, // srcStageMask
                 VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,         // dstStageMask
                 VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,          // srcAccessMask
                 VK_ACCESS_SHADER_READ_BIT,                     // dstAccessMask
                 VK_DEPENDENCY_BY_REGION_BIT                    // dependencyFlags
             }}};

        VkRenderPassCreateInfo render_pass_info{
            VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,           // sType
            nullptr,                                             // pNext
            0,                                                   // flags
            static_cast<uint32_t>(attchmentDescriptions.size()), // attachmentCount
            attchmentDescriptions.data(),                        // pAttachments
            1,                                                   // subpassCount
            &subpass,                                            // pSubpasses
            static_cast<uint32_t>(dependencies.size()),          // dependencyCount
            dependencies.data()                                  // pDependencies
        };
        VK_ASSERT_SUCCESS(vkCreateRenderPass(hs->vk_device, &render_pass_info, nullptr, &app_pass))
    }

    /**
     * @brief Creates a render pass for timewarp.
     */
    void create_timewarp_pass() {
        std::array<VkAttachmentDescription, 1> attchmentDescriptions{{{
            0,                                // flags
            hs->image_format,                   // format
            VK_SAMPLE_COUNT_1_BIT,            // samples
            VK_ATTACHMENT_LOAD_OP_CLEAR,      // loadOp
            VK_ATTACHMENT_STORE_OP_STORE,     // storeOp
            VK_ATTACHMENT_LOAD_OP_DONT_CARE,  // stencilLoadOp
            VK_ATTACHMENT_STORE_OP_DONT_CARE, // stencilStoreOp
            VK_IMAGE_LAYOUT_UNDEFINED,        // initialLayout
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL   // finalLayout
        }}};

        VkAttachmentReference color_attachment_ref{
            0,                                       // attachment
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL // layout
        };

        VkSubpassDescription subpass = {
            0,                               // flags
            VK_PIPELINE_BIND_POINT_GRAPHICS, // pipelineBindPoint
            0,                               // inputAttachmentCount
            nullptr,                         // pInputAttachments
            1,                               // colorAttachmentCount
            &color_attachment_ref,           // pColorAttachments
            nullptr,                         // pResolveAttachments
            nullptr,                         // pDepthStencilAttachment
            0,                               // preserveAttachmentCount
            nullptr                          // pPreserveAttachments
        };

        VkRenderPassCreateInfo render_pass_info{
            VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,           // sType
            nullptr,                                             // pNext
            0,                                                   // flags
            static_cast<uint32_t>(attchmentDescriptions.size()), // attachmentCount
            attchmentDescriptions.data(),                        // pAttachments
            1,                                                   // subpassCount
            &subpass,                                            // pSubpasses
            0,                                                   // dependencyCount
            nullptr                                              // pDependencies
        };

        VK_ASSERT_SUCCESS(vkCreateRenderPass(hs->vk_device, &render_pass_info, nullptr, &timewarp_pass))
    }

    const std::shared_ptr<switchboard>         sb;
    const std::shared_ptr<pose_prediction>     pp;
    const std::shared_ptr<headless_sink>       hs;
    const std::shared_ptr<timewarp>            tw;
    const std::shared_ptr<app>                 src;
    const std::shared_ptr<const RelativeClock> _m_clock;

    VkCommandPool   command_pool{};
    VkCommandBuffer app_command_buffer{};
    VkCommandBuffer timewarp_command_buffer{};

    std::array<VkImage, 2>       depth_images{};
    std::array<VmaAllocation, 2> depth_image_allocations{};
    std::array<VkImageView, 2>   depth_image_views{};

    std::array<VkImage, 2>       offscreen_images{};
    std::array<VmaAllocation, 2> offscreen_image_allocations{};
    std::array<VkImageView, 2>   offscreen_image_views{};
    std::array<VkFramebuffer, 2> offscreen_framebuffers{};

    VkFramebuffer framebuffer;

    VkRenderPass app_pass{};
    VkRenderPass timewarp_pass{};

    VkSemaphore image_available_semaphore{};
    VkSemaphore app_render_finished_semaphore{};
    VkSemaphore timewarp_render_finished_semaphore{};
    VkFence     frame_fence{};
    VkFence     copy_frame_fence{};

    uint64_t timeline_semaphore_value = 1;

    int        fps{};
    time_point last_fps_update;

    int frame_count = 0;
};
PLUGIN_MAIN(native_renderer)
