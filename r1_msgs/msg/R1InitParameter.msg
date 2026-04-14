# red or blue（必須）
string zone
# 要素数3（必須）
int32[] r1_kfs_value
# 要素数4（任意）、使わない場合は-1を入れる
int32[] r2_kfs_value
# 要素数1（任意）、使わない場合は-1を入れる
int32[] r2_fake_kfs_value
# どのKFSを回収するかを自動判別するか
bool enable_auto_select