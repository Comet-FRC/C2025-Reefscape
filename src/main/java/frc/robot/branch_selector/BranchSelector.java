package frc.robot.branch_selector;

public class BranchSelector {
    private char selectedBranch;
    private int level;

    public BranchSelector() {
        this.selectedBranch = 'A';
        this.level = 4;
    }

    public char nextBranch() {
        ++this.selectedBranch;
        if (this.selectedBranch > 'L')
            this.selectedBranch = 'A';
        return this.selectedBranch;
    }

    public char prevBranch() {
        --this.selectedBranch;
        if (this.selectedBranch < 'A')
            this.selectedBranch = 'L';
        return this.selectedBranch;
    }

    public int nextLevel() {
        if (this.level < 4)
            ++this.level;
        return this.level;
    }

    public int prevLevel() {
        if (this.level > 2)
            --this.level;
        return this.level;
    }
}