import pygame
from moviepy.editor import VideoFileClip

# 비디오 파일 경로
video_path = "~/Downloads/test.mp4"

# 비디오 클립 로드
clip = VideoFileClip(video_path)

# pygame 초기화
pygame.init()

# 비디오 크기 가져오기
screen_width, screen_height = clip.size

# 전체화면 설정
screen = pygame.display.set_mode((screen_width, screen_height), pygame.FULLSCREEN)
clock = pygame.time.Clock()

# 비디오 재생
playing = True
while playing:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (
            event.type == pygame.KEYDOWN and event.key == pygame.K_q
        ):
            playing = False

    # 비디오 프레임을 가져와서 화면에 표시
    for frame in clip.iter_frames(fps=clip.fps, dtype="uint8"):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_q
            ):
                playing = False
                break
        if not playing:
            break

        frame_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
        screen.blit(frame_surface, (0, 0))
        pygame.display.update()
        clock.tick(clip.fps)

    # 비디오가 끝나면 다시 처음부터 재생
    clip.reader.initialize()

pygame.quit()
