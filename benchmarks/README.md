# Compile-time benchmarks

`typelist_compile_time.py` measures what the typelist algorithms cost the
compiler. It generates a header with N types (`type_0`, `type_1`, ...)
and a list of them, one translation unit per operation that instantiates
the operation 16 times on lists of its own, compiles every unit with every
mtl include directory given and reports per unit the best-of-3 CPU
seconds, the peak RSS and GCC's GGC allocation total (`-ftime-report`),
which is deterministic and the figure to compare on a noisy machine.

```
benchmarks/typelist_compile_time.py                       # the checked-out mtl
benchmarks/typelist_compile_time.py --mtl main=../mtl-main/include --mtl work=include
benchmarks/typelist_compile_time.py --ops filter,unique --sizes 100,400
benchmarks/typelist_compile_time.py --cxx arm-zephyr-eabi-g++
```

The default sizes (37, 150, 600) are deliberately no powers of two; 150
is the order of a USB PD policy engine's transition table. "does not
compile" marks the default template instantiation depth (900) exceeded.

## Results

2026-10-03, GCC 15.2.0 x86-64, `-std=c++20 -fsyntax-only`, 16 instances,
best of 3. `d49b48b` is the last commit before the algorithms were
reworked, `514326c` the reworked ones (concat of any number of lists,
filter/remove_if/count_if/has_a over the pack, unique in one forward
pass).

```
                                  d49b48b                     514326c
                  cpu s   rss MB   ggc MB     cpu s   rss MB   ggc MB
has_a 37           0.06       35       11      0.05       35       10
has_a 150          0.07       38       14      0.06       36       11
has_a 600          0.15       58       39      0.08       41       18
count_if 37        0.08       41       17      0.06       37       12
count_if 150       0.23       72       54      0.11       44       20
count_if 600             does not compile      0.30       72       51
all_of 37          0.05       35       11      0.05       36       11
all_of 150         0.06       38       13      0.06       38       13
all_of 600         0.09       47       24      0.09       47       25
any_of 37          0.05       35       11      0.06       35       11
any_of 150         0.06       38       14      0.06       38       14
any_of 600         0.15       58       38      0.15       57       39
none_of 37         0.05       35       11      0.05       36       11
none_of 150        0.07       39       14      0.07       39       15
none_of 600        0.16       61       42      0.16       61       42
find_if 37         0.05       36       11      0.05       36       11
find_if 150        0.07       40       16      0.07       40       16
find_if 600              does not compile            does not compile
filter 37          0.10       47       23      0.08       39       15
filter 150         0.36      102       89      0.20       55       31
filter 600               does not compile      1.01      173      157
remove_if 37       0.16       49       25      0.10       39       15
remove_if 150      0.56      120      115      0.21       56       33
remove_if 600            does not compile      1.15      187      178
transform 37       0.06       35       10      0.06       35       10
transform 150      0.07       36       12      0.07       36       12
transform 600      0.10       41       18      0.10       41       18
reverse 37         0.07       36       11      0.07       36       11
reverse 150        0.10       41       18      0.10       41       19
reverse 600              does not compile            does not compile
index_of 37        0.07       35       11      0.07       35       11
index_of 150       0.09       39       15      0.09       39       15
index_of 600             does not compile            does not compile
at 37              0.06       35       10      0.06       35       10
at 150             0.07       35       11      0.07       35       11
at 600             0.09       38       16      0.09       38       16
back 37            0.07       36       11      0.07       35       11
back 150           0.10       41       17      0.10       41       17
back 600                 does not compile            does not compile
unique 37          1.58      334      338      0.26       71       50
unique 150        46.48     7480     8940      2.10      425      453
unique 600               does not compile            does not compile
linearize 37       0.09       36       12      0.08       36       12
linearize 150      0.11       43       20      0.11       43       20
linearize 600            does not compile            does not compile
concat_2 37        0.07       35       10      0.07       35       10
concat_2 150       0.07       35       11      0.07       36       11
concat_2 600       0.09       39       18      0.09       39       17
concat_3 37              does not compile      0.07       36       11
concat_3 150             does not compile      0.08       37       13
concat_3 600             does not compile      0.12       42       21
concat_5 37              does not compile      0.07       36       11
concat_5 150             does not compile      0.08       37       14
concat_5 600             does not compile      0.13       44       26
sort 37            0.32       77       55      0.35       82       60
sort 64            0.77      146      134      0.84      157      145
```

(`unique` doubles its list, so its 150 is 300 elements; `concat_3` and
`concat_5` did not exist before: concat took two lists.)

### What it buys a user

The USB Type-C stack's host tests, each a translation unit instantiating
several machines (GCC 15.2, `-std=gnu++23 -O1 -fno-rtti`, best of 2;
the mtl commits between the two columns also rework the machine itself:
observers visited by index, observing hooks only for notified states,
one trait per dispatch instead of three per state):

```
                        d49b48b                     514326c
                  cpu s   rss MB   ggc MB     cpu s   rss MB   ggc MB
pd_drp             36.2     2229     3663       24.6     1337     2339
compliance         18.9     1919     2633        5.6      610      806
pd_single_role     22.6     1381     2341       14.2      876     1371
policy_engine      20.8     1238     2072       14.1      839     1338
policy_engine_source 21.4   1277     2153       14.0      846     1344
type_c_drp         22.5     1306     2145       16.0      826     1434
```

Clean build of the whole test suite, 8 jobs: 84 s wall / 492 s CPU ->
53 s / 293 s. Functions emitted for `pd_drp.cpp` at -O0: 61 322 ->
41 497. The `pd_drp` Zephyr sample (stm32g081b_eval, arm-gcc 12, -Os):
63 316 -> 63 076 B flash, RAM unchanged.

### Design notes the numbers settled

- `concat` merges sixteen lists per instantiation and pads three to
  fifteen up to sixteen; two merge directly. Exactly one partial
  specialization matches any count: GCC deduces every matching
  specialization before it orders them, so a ladder of 2/4/8/16-wide
  steps measured slower than the sixteen alone (filter 600: 158 vs 89 MB
  GGC in an earlier run), and padding instead of a pairwise remainder
  costs nothing extra.
- A step's cost grows with the elements merged so far, so what counts is
  the number of steps carrying a long accumulator, not the number of
  specializations tried: arity mismatches fail before deduction.
