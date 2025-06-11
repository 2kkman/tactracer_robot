import cv2
import numpy as np
from scipy.spatial.distance import cosine
from sklearn.metrics.pairwise import cosine_similarity
import matplotlib.pyplot as plt
from skimage.metrics import structural_similarity as ssim
from skimage.feature import local_binary_pattern
import hashlib

class AdvancedImageSimilarity:
    def __init__(self):
        # AKAZE는 SIFT보다 더 안정적이고 빠름
        self.akaze = cv2.AKAZE_create()
        # BRISK도 좋은 대안
        self.brisk = cv2.BRISK_create()
        
    def method1_image_registration_similarity(self, img1, img2):
        """이미지 정합을 통한 유사도 계산 - 손떨림에 매우 강함"""
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        # AKAZE 특징점 검출 (SIFT보다 더 안정적)
        kp1, des1 = self.akaze.detectAndCompute(gray1, None)
        kp2, des2 = self.akaze.detectAndCompute(gray2, None)
        
        if des1 is None or des2 is None or len(des1) < 10 or len(des2) < 10:
            return self._fallback_correlation(gray1, gray2)
        
        # 매칭
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)
        
        # 좋은 매치 필터링 (더 관대한 임계값 사용)
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.8 * n.distance:  # 0.7에서 0.8로 완화
                    good_matches.append(m)
        
        if len(good_matches) < 8:  # 호모그래피 계산을 위한 최소 점수
            return self._fallback_correlation(gray1, gray2)
        
        # 호모그래피 계산으로 이미지 정합
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        try:
            M, mask = cv2.findHomography(src_pts, dst_pts, 
                                       cv2.RANSAC, 5.0)  # 더 관대한 임계값
            
            if M is None:
                return self._fallback_correlation(gray1, gray2)
            
            # 이미지 워핑
            h, w = gray2.shape
            warped = cv2.warpPerspective(gray1, M, (w, h))
            
            # 워핑된 이미지와 원본 비교
            similarity = ssim(warped, gray2)
            
            # 인라이어 비율도 고려
            inlier_ratio = np.sum(mask) / len(mask) if mask is not None else 0
            
            # 최종 점수는 SSIM과 인라이어 비율의 조합
            final_score = 0.7 * similarity + 0.3 * inlier_ratio
            return max(0, min(1, final_score))
            
        except:
            return self._fallback_correlation(gray1, gray2)
    
    def _fallback_correlation(self, gray1, gray2):
        """특징점이 부족할 때 사용하는 대안 방법"""
        # 크기 정규화
        h, w = min(gray1.shape[0], gray2.shape[0]), min(gray1.shape[1], gray2.shape[1])
        gray1_resized = cv2.resize(gray1, (w, h))
        gray2_resized = cv2.resize(gray2, (w, h))
        
        # 정규화된 상관계수
        correlation = cv2.matchTemplate(gray1_resized, gray2_resized, cv2.TM_CCOEFF_NORMED)[0][0]
        return max(0, correlation)
    
    def method2_phase_correlation(self, img1, img2):
        """위상 상관을 이용한 유사도 - 평행이동에 매우 강함"""
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        # 크기 맞추기
        h, w = min(gray1.shape[0], gray2.shape[0]), min(gray1.shape[1], gray2.shape[1])
        gray1 = cv2.resize(gray1, (w, h))
        gray2 = cv2.resize(gray2, (w, h))
        
        # FFT
        f1 = np.fft.fft2(gray1.astype(np.float32))
        f2 = np.fft.fft2(gray2.astype(np.float32))
        
        # 위상 상관
        cross_power_spectrum = (f1 * np.conj(f2)) / (np.abs(f1 * np.conj(f2)) + 1e-10)
        correlation = np.fft.ifft2(cross_power_spectrum)
        
        # 최대 상관값
        max_corr = np.max(np.abs(correlation))
        
        return min(1.0, max_corr)
    
    def method3_structural_similarity(self, img1, img2):
        """구조적 유사도 지수 (SSIM) - 손떨림에 어느 정도 강함"""
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        # 크기 맞추기
        h, w = min(gray1.shape[0], gray2.shape[0]), min(gray1.shape[1], gray2.shape[1])
        gray1 = cv2.resize(gray1, (w, h))
        gray2 = cv2.resize(gray2, (w, h))
        
        # SSIM 계산 (윈도우 크기를 키워서 더 관대하게)
        similarity = ssim(gray1, gray2, win_size=11)  # 기본값 7에서 11로
        
        return max(0, similarity)
    
    def method4_local_binary_pattern(self, img1, img2):
        """LBP (Local Binary Pattern) 기반 유사도 - 조명 변화에 강함"""
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        # 크기 맞추기
        h, w = min(gray1.shape[0], gray2.shape[0]), min(gray1.shape[1], gray2.shape[1])
        gray1 = cv2.resize(gray1, (w, h))
        gray2 = cv2.resize(gray2, (w, h))
        
        # LBP 계산
        radius = 3
        n_points = 8 * radius
        
        lbp1 = local_binary_pattern(gray1, n_points, radius, method='uniform')
        lbp2 = local_binary_pattern(gray2, n_points, radius, method='uniform')
        
        # 히스토그램 비교
        hist1, _ = np.histogram(lbp1.ravel(), bins=n_points + 2, range=(0, n_points + 2))
        hist2, _ = np.histogram(lbp2.ravel(), bins=n_points + 2, range=(0, n_points + 2))
        
        # 정규화
        hist1 = hist1.astype(float) / (hist1.sum() + 1e-10)
        hist2 = hist2.astype(float) / (hist2.sum() + 1e-10)
        
        # 코사인 유사도
        similarity = 1 - cosine(hist1, hist2)
        return max(0, similarity)
    
    def method5_perceptual_hash(self, img1, img2):
        """지각적 해시 비교 - 매우 빠르고 실용적"""
        def dhash(image, hash_size=8):
            # 그레이스케일 변환
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            # 리사이즈
            resized = cv2.resize(gray, (hash_size + 1, hash_size))
            # 차이 계산
            diff = resized[:, 1:] > resized[:, :-1]
            return sum([2 ** i for (i, v) in enumerate(diff.flatten()) if v])
        
        hash1 = dhash(img1)
        hash2 = dhash(img2)
        
        # 해밍 거리 계산
        hamming_distance = bin(hash1 ^ hash2).count('1')
        
        # 유사도로 변환 (0-64 범위를 0-1로)
        similarity = 1 - (hamming_distance / 64.0)
        return similarity
    
    def method6_multi_scale_correlation(self, img1, img2):
        """다중 스케일 상관 계수 - 손떨림과 약간의 크기 변화에 강함"""
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
        
        max_similarity = 0
        scales = [0.8, 0.9, 1.0, 1.1, 1.2]
        
        for scale in scales:
            # 이미지 크기 조정
            h1, w1 = gray1.shape
            new_h, new_w = int(h1 * scale), int(w1 * scale)
            
            if new_h <= 0 or new_w <= 0:
                continue
                
            scaled_img1 = cv2.resize(gray1, (new_w, new_h))
            
            # 공통 크기로 맞추기
            h_common = min(scaled_img1.shape[0], gray2.shape[0])
            w_common = min(scaled_img1.shape[1], gray2.shape[1])
            
            if h_common <= 10 or w_common <= 10:
                continue
            
            cropped_img1 = scaled_img1[:h_common, :w_common]
            cropped_img2 = gray2[:h_common, :w_common]
            
            # 정규화된 상관계수
            corr = cv2.matchTemplate(cropped_img1, cropped_img2, cv2.TM_CCOEFF_NORMED)
            if corr.size > 0:
                max_similarity = max(max_similarity, corr[0, 0])
        
        return max(0, max_similarity)
    
    def method7_advanced_combined(self, img1, img2):
        """고급 통합 방법 - 손떨림에 특화된 가중치"""
        methods_scores = {}
        
        # 각 방법별 점수 계산
        methods_scores['registration'] = self.method1_image_registration_similarity(img1, img2)
        methods_scores['phase_corr'] = self.method2_phase_correlation(img1, img2)
        methods_scores['ssim'] = self.method3_structural_similarity(img1, img2)
        methods_scores['lbp'] = self.method4_local_binary_pattern(img1, img2)
        methods_scores['phash'] = self.method5_perceptual_hash(img1, img2)
        methods_scores['multi_corr'] = self.method6_multi_scale_correlation(img1, img2)
        
        # 손떨림에 특화된 가중치 (등록과 위상상관에 높은 가중치)
        weights = {
            'registration': 0.25,  # 가장 중요
            'phase_corr': 0.20,    # 평행이동에 강함
            'ssim': 0.20,          # 구조적 유사성
            'multi_corr': 0.15,    # 다중 스케일
            'lbp': 0.10,           # 조명 변화 대응
            'phash': 0.10          # 빠른 판별
        }
        
        # 가중 평균
        final_score = sum(methods_scores[method] * weights[method] 
                         for method in methods_scores)
        
        return final_score, methods_scores
    
    def quick_similarity_check(self, img1, img2, threshold=0.7):
        """빠른 유사도 체크 - 실시간 용도"""
        # 1단계: 빠른 해시 비교
        hash_sim = self.method5_perceptual_hash(img1, img2)
        if hash_sim < 0.5:  # 너무 다르면 바로 리턴
            return hash_sim, "다른 이미지"
        
        # 2단계: 위상 상관
        phase_sim = self.method2_phase_correlation(img1, img2)
        combined_sim = 0.4 * hash_sim + 0.6 * phase_sim
        
        if combined_sim > threshold:
            return combined_sim, "같은 장소 (손떨림)"
        else:
            return combined_sim, "다른 장소"
    
    def compare_images(self, img1_path, img2_path, method='advanced_combined'):
        """메인 비교 함수"""
        img1 = cv2.imread(img1_path)
        img2 = cv2.imread(img2_path)
        
        if img1 is None or img2 is None:
            raise ValueError("이미지를 로드할 수 없습니다.")
        
        methods = {
            'registration': self.method1_image_registration_similarity,
            'phase_correlation': self.method2_phase_correlation,
            'ssim': self.method3_structural_similarity,
            'lbp': self.method4_local_binary_pattern,
            'phash': self.method5_perceptual_hash,
            'multi_correlation': self.method6_multi_scale_correlation,
            'advanced_combined': self.method7_advanced_combined,
            'quick': lambda x, y: self.quick_similarity_check(x, y)
        }
        
        if method not in methods:
            raise ValueError(f"지원하지 않는 방법: {method}")
        
        result = methods[method](img1, img2)
        return result

# 사용 예제
def test_with_hand_shake_images():
    """손떨림 이미지 테스트"""
    calculator = AdvancedImageSimilarity()
    
    img1_path = "/root/Downloads/crop1.jpg"  # 첫 번째 이미지
    img2_path = "/root/Downloads/crop2.jpg"  # 첫 번째 이미지
    #img2_path = "image2.jpg"  # 손떨림이 있는 두 번째 이미지
    
    print("=== 손떨림 이미지 유사도 분석 ===")
    
    # 개별 방법들 테스트
    methods = [
        ('registration', '이미지 정합'),
        ('phase_correlation', '위상 상관'),
        ('ssim', '구조적 유사도'),
        ('lbp', 'LBP 패턴'),
        ('phash', '지각적 해시'),
        ('multi_correlation', '다중 스케일 상관')
    ]
    
    for method, name in methods:
        try:
            similarity = calculator.compare_images(img1_path, img2_path, method)
            print(f"{name}: {similarity:.4f}")
        except Exception as e:
            print(f"{name}: 오류 - {e}")
    
    # 통합 방법
    print("\n=== 통합 분석 ===")
    try:
        final_score, detailed = calculator.compare_images(img1_path, img2_path, 'advanced_combined')
        print(f"최종 유사도: {final_score:.4f}")
        
        print("\n세부 점수:")
        for method, score in detailed.items():
            print(f"  {method}: {score:.4f}")
        
        # 결과 해석
        if final_score > 0.8:
            print("\n결론: 같은 장소의 이미지 (손떨림 수준)")
        elif final_score > 0.6:
            print("\n결론: 유사한 장소의 이미지")
        else:
            print("\n결론: 다른 장소의 이미지")
            
    except Exception as e:
        print(f"통합 분석 오류: {e}")
    
    # 빠른 체크
    print("\n=== 빠른 체크 ===")
    try:
        quick_result, interpretation = calculator.compare_images(img1_path, img2_path, 'quick')
        print(f"빠른 판정: {quick_result:.4f} - {interpretation}")
    except Exception as e:
        print(f"빠른 체크 오류: {e}")

if __name__ == "__main__":
    test_with_hand_shake_images()