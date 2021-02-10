std_msgs/Header header
int8 id                 # car id
bool[] mask             # validity : 1 or 0
int8 valid_sequence_num # ex) valid_sequence_num=4 means that only first 4 sequences are valid
Object[] trajectory     # 