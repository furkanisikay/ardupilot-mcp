# ArduPilot MCP — Log Teşhis Beyni — Tasarım Dokümanı

- **Tarih:** 2026-06-14
- **Durum:** Onaylandı (brainstorming sonucu) → planlamaya hazır
- **Çalışma adı:** `ardupilot-mcp` (ürün adı sonra)

---

## 1. Bağlam & Motivasyon

Amaç: bir LLM'i (Claude Desktop vb.) **güvenilir bir ArduPilot uçuş-logu teşhisçisine** dönüştüren açık kaynak bir MCP (Model Context Protocol) server'ı. Kullanıcı bir DataFlash `.bin` logu gösterir, server deterministik bir kontrol motoruyla logu analiz eder, LLM bulguları insan diline çevirir ve takip sorularında derine iner.

### Dürüst konumlandırma (prior art)
"Dünyada ilk ArduPilot/PX4 MCP server'ı" iddiası **geçersizdir** (yüksek güvenle doğrulandı) ve kullanılmayacaktır. Mevcut prior art:

- **DroneServer** (`PeterJBurke/droneserver`) — çift platform (PX4 + ArduPilot) MCP server, ~45 araç, v1.4.0 (Ara 2025). Canlı kontrole odaklı.
- **arXiv:2601.15486** (Ramos-Silva & Burke, Oca 2026) — DroneServer'ın makalesi; "ilk universal drone-agnostic" MCP arayüzü iddiasını çoktan almış.
- **MAVLinkMCP** (`ion-g-ion/MAVLinkMCP`, Nis 2025) — ilk MAVLink MCP server. MAVLink ortak olduğu için her MAVLink MCP'si zaten iki platformu da konuşur.
- **ardupilot-mcp-server-sandbox** (`hfujikawa77`, Nis 2025) ve **EchoPilot** (PX4, Ağu 2025) — tek-platform sürümler.

Bu projeler canlı **kontrole** odaklı ve çok genç/olgunlaşmamış (gerçek GitHub yıldızları tek haneli–~26). README bu projeleri prior art olarak açıkça anacaktır.

### Farklılaşma (savunulabilir manşet)
> "ArduPilot uçuş loglarını **deterministik-temelli** olarak teşhis eden, genişletilebilir check framework'üne sahip açık kaynak MCP server'ı."

Üç sütun: (1) **deterministik motor temellemesi** — topluluğun "LLM güvenilmez" eleştirisini doğrudan çürütür; (2) **teşhis derinliği** (canlı kontrol değil); (3) **genişletilebilir check framework'ü** — "AI log teşhisinin platformu" potansiyeli.

---

## 2. Hedefler / Hedef-olmayanlar

### Hedefler (MVP)
- ArduPilot DataFlash `.bin` loglarını offline analiz et.
- **Araç tipi önceliği: ArduCopter.** Check'lerin çoğu (notch, attitude takibi, motor dengesizliği) copter-merkezli. Parser ve framework araç-agnostik tasarlanır; Plane/Rover/Sub check'leri sonraki fazlara bırakılır ama mimari engellemez.
- Deterministik bir **check registry** (plugin mimarisi) + çekirdek teşhis check seti.
- **Tuning tavsiyesi** (sadece öneri, yazma yok): harmonik notch (FFT), PID/AUTOTUNE yorumlama.
- Hibrit motor×LLM: yapısal `DiagnosisReport` + LLM'in ham loga inebileceği sorgu araçları.
- FastMCP (Python) + stdio transport → Claude Desktop.

### Hedef-olmayanlar (kapsam dışı, sonraki fazlar)
- PX4 / ULog desteği (ayrı parser + kural seti).
- Canlı MAVLink bağlantısı / canlı telemetri / araçtan log indirme.
- Tuning'in **uygulanması** (parametre yazma) — herhangi bir aktüasyon.
- Topluluk check paylaşım/dağıtım ekosistemi (registry hazır ama paylaşım altyapısı sonra).

---

## 3. Kararlar Günlüğü (brainstorming çıktısı)

| # | Karar | Gerekçe |
|---|-------|---------|
| 1 | "Dünyada ilk" iddiasını bırak; "en güvenilir/derin teşhis" konumlan | Prior art doğrulandı; dürüstlük + savunulabilirlik |
| 2 | Yön: **Log/teşhis beyni** (canlı kontrol değil) | En yüksek değer/risk; ~sıfır güvenlik riski; topluluk talebi |
| 3 | Platform: **ArduPilot önce** (.bin/DFReader) | En yüksek log-teşhis talebi (GSoC 2026, resmi Log Analyzer AI); klasör adı |
| 4 | Girdi: **sadece offline log** (canlı yok) | En yalın/güvenli MVP; MAVLink link katmanı gerekmez |
| 5 | Motor×LLM: **hibrit** (motor + sorgu araçları) | Sağlam taban + takip-sorusu derinliği = gerçek "beyin" |
| 6 | Kapsam: **tam çerçeve** (registry + çekirdek check + tuning) | Genişletilebilir, gelecek-korumalı, "lider/platform" iddiası |
| 7 | Stack: **Python + FastMCP**, pymavlink **DFReader**, stdio | pymavlink/MCP Python-first; stdio en basit (Claude Desktop) |

---

## 4. Mimari (katmanlı)

```
.bin dosyası
   │
   ▼
[1] Parser  ── pymavlink DFReader ──▶ FlightLog (tipli, indeksli, lazy)
   │                                   meta: araç tipi, firmware, süre, PARM dump
   ▼
[2] Check Registry  ◀── @register_check ile plugin'ler (gevşek bağlı, bağımsız test)
   │   her Check: gerekli mesajları bildirir → Finding'ler üretir
   ▼
[3] Diagnosis Orchestrator ── tüm check'leri koştur, önem-sırala ─▶ DiagnosisReport
   │
   ▼
[4] MCP Tool Layer (FastMCP, stdio) ── hepsi readOnly, Pydantic outputSchema
   │
   ▼
[5] LLM (Claude Desktop) ── analyze_log ile temellenir, sorgu araçlarıyla derine iner, açıklar
```

**Anahtar ilke:** deterministik motor **tek doğruluk kaynağı**; LLM yorumlar, üretmez. Her katman bağımsız anlaşılır ve test edilebilir; iyi tanımlı arayüzlerle konuşur.

### Katman sorumlulukları
- **[1] Parser** — `.bin` → `FlightLog`. Mesaj tiplerine lazy/indeksli erişim (ATT, GPS, ERR, MODE, EV, VIBE, BAT, XKF/EKF, RCOU, IMU, PARM, MSG, FMT). Meta veri çıkarımı (araç tipi, firmware sürümü, süre, board, PARM dump). Bozuk/kesik loglara karşı savunmacı.
- **[2] Check Registry** — `@register_check` dekoratörüyle kaydedilen `Check` birimleri. Her check ihtiyaç duyduğu mesaj/parametreleri bildirir, `FlightLog`'a karşı koşar, sıfır+ `Finding` üretir. Yeni check eklemek = bir dosya bırakmak.
- **[3] Diagnosis Orchestrator** — check'leri (tümü ya da seçili alt küme) koşturur, `Finding`'leri toplar, önem sıralar, `DiagnosisReport` + deterministik özet iskeleti üretir. Koşamayan check'leri (eksik veri) sebebiyle birlikte raporlar.
- **[4] MCP Tool Layer** — FastMCP, stdio. ~7 tool, hepsi `readOnlyHint`. Pydantic `outputSchema` ile doğrulanmış `structuredContent`.
- **[5] LLM** — istemci (Claude Desktop). `analyze_log` ile temellenir, sorgu araçlarıyla eşeler, açıklar.

---

## 5. Veri Modeli (taslak)

```python
# Parser çıktısı
class FlightLog:
    path: str
    vehicle_type: str            # Copter / Plane / Rover / Sub ...
    firmware_version: str | None
    board: str | None
    start_time: float            # log içi zaman tabanı (TimeUS)
    duration_s: float
    available_messages: set[str] # logda bulunan mesaj tipleri
    params: dict[str, float]     # PARM dump
    integrity: LogIntegrity      # ok / truncated / corrupt-partial
    def get(msg_type, fields?, time_range?) -> Iterable[record]   # lazy/indeksli

class Severity(Enum): INFO; WARN; CRITICAL

class Finding:
    check_id: str
    severity: Severity
    title: str
    explanation: str             # deterministik, insan-okur
    evidence: Evidence           # time_range, örnek değerler, kaynak mesaj tipleri
    recommendation: str | None   # tuning/teşhis önerisi (sadece metin)

class CheckResult:
    check_id: str
    status: Literal["ran", "skipped", "error"]
    skipped_reason: str | None   # ör. "VIBE mesajı loglanmamış"
    findings: list[Finding]

class DiagnosisReport:
    log_summary: LogSummary
    results: list[CheckResult]
    findings_by_severity: dict[Severity, list[Finding]]
    summary_text: str            # deterministik özet iskeleti (LLM bunu zenginleştirir)
    checks_skipped: list[tuple[str, str]]  # (check_id, sebep)
```

### Check arayüzü
```python
class Check(Protocol):
    id: str
    title: str
    requires: set[str]           # gerekli mesaj tipleri (ör. {"ATT"})
    requires_params: set[str]    # gerekli parametreler (opsiyonel)
    def run(self, log: FlightLog) -> list[Finding]: ...

@register_check
class AttitudeTrackingCheck: ...
```
Orchestrator, `requires` bir logda yoksa check'i koşturmaz; `CheckResult(status="skipped", reason=...)` üretir.

---

## 6. Check Kataloğu

### Çekirdek teşhis (MVP)
- **Olaylar & hatalar** — ERR alt-sistem/ECode çözümü, MODE değişimleri, EV olayları, STATUSTEXT/MSG uyarıları, failsafe tetikleri.
- **EKF/estimator sağlığı** — innovation/varyans spike'ları, XKF bayrakları, GPS glitch, pozisyon/hız varyansı.
- **Titreşim** — VIBE clipping (clip0/1/2), VibeX/Y/Z eşikleri.
- **Güç** — yük altında batarya voltaj sag'i, akım spike'ları, failsafe voltajı, ani voltaj düşüşleri.
- **GPS** — HDOP, uydu sayısı, fix-type düşüşleri, pozisyon sıçramaları.
- **Pusula/mag** — MAG tutarsızlığı, alan şiddeti anomalileri, pusula-vs-GPS yön.
- **Attitude takibi** (*taç mücevheri*) — ATT.DesRoll/DesPitch/DesYaw vs gerçek sapması → mekanik arıza / yetersiz güç / yanlış tuning / motor saturasyonu (RCOU ile çapraz).
- **Motor dengesizliği** — RCOU çıkış yayılımı (bir motor çok daha fazla çalışıyor → dengesizlik/mekanik).
- **Zamanlama/performans** — PM load, scheduler overrun, log boşlukları.

### Tuning tavsiyesi (sadece öneri)
- **Harmonik notch** — IMU gyro verisinin FFT'si → motor gürültü tepe frekansı → `INS_HNTCH_FREQ/BW/REF` önerisi.
- **PID değerlendirme** — ATT takip + rate loglarından eksen bazlı over/under-damped tespiti, yön önerisi.
- **AUTOTUNE yorumlama** — sonuç parametrelerinin makullük değerlendirmesi.

---

## 7. MCP Araç Yüzeyi (~7 tool, hepsi `readOnlyHint: true`)

| Tool | İmza (kavramsal) | Çıktı |
|------|------------------|-------|
| `analyze_log` | `(path)` | `DiagnosisReport` (önem-sıralı bulgular + özet). **Manşet.** |
| `log_summary` | `(path)` | `LogSummary` (araç, firmware, süre, uçulan modlar, max irtifa). Ucuz oryantasyon. |
| `list_events` | `(path, kinds?, time_range?)` | Çözümlenmiş ERR/MODE/EV/MSG/failsafe zaman çizelgesi. |
| `query_timeseries` | `(path, msg_type, fields, time_range?, downsample?)` | Ham/aggregat sayısal seri. Context şişmesini önlemek için downsample. |
| `get_params` | `(path, name_glob?)` | Logdaki parametre değerleri. |
| `recommend_tuning` | `(path, area?)` | Tuning önerileri (notch/PID/AUTOTUNE). **Sadece metin, yazma yok.** |
| `list_checks` | `()` | Registry'deki check'leri tanıt (framework konumlandırması). |

Tüm tool'lar `outputSchema` (Pydantic) bildirir; doğrulanmış `structuredContent` + metin aynası döner. Tool hataları `isError`.

---

## 8. Veri Akışı

1. LLM `analyze_log(path)` çağırır.
2. Parser `.bin`'i `FlightLog`'a çözer (lazy/indeksli).
3. Orchestrator registry'deki check'leri koşturur; eksik-veri check'lerini atlar.
4. `Finding`'ler toplanır, önem sıralanır → `DiagnosisReport`.
5. MCP doğrulanmış `structuredContent` döner.
6. LLM raporu narate eder; kullanıcı takip sorarsa `query_timeseries` / `list_events` / `get_params` ile derine iner.

---

## 9. Hata Yönetimi
- **Bozuk/kesik `.bin`** (kaza logları!) — savunmacı parse → kısmi `FlightLog` + `log_integrity` bulgusu. Çökmüş drone'un logu tam da analiz edilmesi gereken logdur.
- **Eksik mesaj tipleri** (LOG_BITMASK'a bağlı) — check `skipped` döner, hata değil; hangi check neden koşamadı raporlanır.
- **Bilinmeyen/eski firmware şeması** — DFReader self-describing (FMT); zarif idare, firmware sürümü yüzeye çıkar.
- **Büyük loglar (100+ MB)** — indeksli/lazy parse; timeseries'te downsample (LLM context'i için küçük payload).
- **Dosya yok / yanlış format / desteklenmeyen araç** — net `isError` yanıtı.

---

## 10. Test Stratejisi (TDD)
- Her `Check` küçük **fixture `.bin`** ile bağımsız unit-test edilir (bilinen titreşim çökmesi, batarya failsafe, pusula hatası, EKF arızası içeren gerçek log parçaları).
- **Golden-file** testleri: bilinen log → beklenen `DiagnosisReport`.
- **Parser** testleri: bozuk/kesik loglar, eksik mesaj tipleri, çoklu araç tipi (Copter/Plane/Rover).
- **MCP entegrasyon** testleri: tool I/O şemaları, `structuredContent` doğrulaması.
- Akış: **önce başarısız testi yaz** (fixture log + beklenen bulgu), sonra check'i implemente et. Registry pattern buna birebir uyar.

---

## 11. Güvenlik
MVP **yapısı gereği salt-okunur**: offline log analizi — aktüasyon yok, parametre yazma yok, ağ erişimi yok. Server fiziksel olarak bir drone'u hareket ettiremez. Tuning yalnızca **tavsiye** üretir. Tüm aktüasyon/güvenlik-gateway tartışması (SITL-varsayılan, elicitation, kill switch, durum makinesi) canlı-bağlantı fazına (kapsam dışı) aittir.

---

## 12. Bağımlılıklar & Teknoloji
- **Python** (3.11+ önerilir).
- **FastMCP** (MCP Python SDK) — `@mcp.tool`, Pydantic, lifespan, stdio transport.
- **pymavlink** — `DFReader` (DataFlash `.bin` parse), mesaj sözlükleri.
- **numpy / scipy** — FFT (notch) ve sinyal/sayısal işleme.
- (Opsiyonel) **pandas** — timeseries aggregasyon/downsample kolaylığı.

---

## 13. Yol Haritası (fazlar)
1. **MVP (bu spec):** ArduPilot offline `.bin` teşhis + tuning tavsiyesi + check framework + 7 tool, stdio.
2. **Faz 2:** PX4/ULog parser + ortak teşhis soyutlaması; daha fazla check.
3. **Faz 3:** Canlı MAVLink bağlantısı (SITL-varsayılan), canlı telemetri Q&A, araçtan log indirme — *tam güvenlik gateway'i ile*.
4. **Faz 4:** Topluluk check paylaşım ekosistemi; tuning'in onay-kapılı uygulanması.

---

## 14. Açık Riskler / Sorular
- **Fixture log temini** — gerçek arızalı `.bin` logları (titreşim/batarya/EKF/pusula) nereden? ArduPilot topluluk log arşivleri + SITL'de kasıtlı arıza üretimi değerlendirilecek (implementasyon planı detaylandıracak).
- **DFReader performansı** büyük loglarda — indeksleme stratejisi profillenmeli.
- **Check eşik değerleri** — ArduPilot'un resmi teşhis kılavuzu + topluluk araçları (ALDA vb.) referans alınacak; eşikler araç tipine göre parametrize edilebilir olmalı.
- **Attitude sapma yorumu** — mekanik arıza vs tuning vs yetersiz güç ayrımı kanıt-temelli olmalı (tek metrikle aşırı-iddia etme).
